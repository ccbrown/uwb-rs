use crate::{Config, Device, Error, ReadyState, Result};

use dw3000_ng::{configs, ll};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::{delay::DelayNs, digital::Wait, spi::SpiDevice};
use smoltcp::wire::ieee802154::Address;

pub struct UninitializedState;

async fn read_otp<SPI: SpiDevice>(dev: &mut ll::DW3000<SPI>, addr: u16) -> Result<SPI, u32> {
    dev.otp_cfg().write(|w| w.otp_man(1)).await?;
    dev.otp_addr().modify(|_, w| w.otp_addr(addr)).await?;
    dev.otp_cfg().write(|w| w.otp_read(1)).await?;
    Ok(dev.otp_rdata().read().await?.value())
}

async fn calibrate_pgf_inner<SPI: SpiDevice, D: DelayNs>(
    dev: &mut ll::DW3000<SPI>,
    mut delay: D,
) -> Result<SPI, ()> {
    dev.rx_cal()
        .modify(|_, w| w.comp_dly(0x2).cal_mode(1))
        .await?;

    dev.rx_cal().modify(|_, w| w.cal_en(1)).await?;

    let mut max_retries = 3;
    let mut success = true;
    delay.delay_us(20).await;
    while dev.rx_cal_sts().read().await?.value() == 0 {
        max_retries -= 1;
        if max_retries == 0 {
            success = false;
            break;
        }
        delay.delay_us(20).await;
    }

    if !success {
        return Err(Error::PgfCalibrationFailed);
    }

    dev.rx_cal().modify(|_, w| w.cal_mode(0).cal_en(0)).await?;
    dev.rx_cal_sts().modify(|_, w| w.value(1)).await?;
    dev.rx_cal()
        .modify(|r, w| w.comp_dly(r.comp_dly() | 0x1))
        .await?;

    let rx_cal_resi = dev.rx_cal_resi().read().await?.value();
    let rx_cal_resq = dev.rx_cal_resq().read().await?.value();
    if rx_cal_resi == 0x1fffffff || rx_cal_resq == 0x1fffffff {
        return Err(Error::PgfCalibrationFailed);
    }

    Ok(())
}

pub(crate) async fn calibrate_pgf<SPI: SpiDevice, D: DelayNs>(
    dev: &mut ll::DW3000<SPI>,
    mut delay: D,
) -> Result<SPI, ()> {
    let ldo_ctrl_low = dev.ldo_ctrl().read().await?.low();
    dev.ldo_ctrl().modify(|_, w| w.low(0x105)).await?;

    delay.delay_us(20).await;

    let pgf_cal_result = calibrate_pgf_inner(dev, delay).await;

    dev.ldo_ctrl().modify(|_, w| w.low(ldo_ctrl_low)).await?;
    pgf_cal_result
}

pub(crate) async fn tune_dgc<SPI: SpiDevice>(
    dev: &mut ll::DW3000<SPI>,
    config: &Config,
) -> Result<SPI, ()> {
    if (9..=24).contains(&config.rx_preamble_code) {
        let dgc_otp = read_otp(dev, 0x20).await?;

        let ch = config.channel;
        if dgc_otp == 0x10000240 {
            dev.otp_cfg()
                .modify(|_, w| w.dgc_sel(ch as u8).dgc_kick(1))
                .await?;
        } else {
            dev.dgc_lut_0()
                .modify(|_, w| w.value(ch.get_recommended_dgc_lut_0()))
                .await?;
            dev.dgc_lut_1()
                .modify(|_, w| w.value(ch.get_recommended_dgc_lut_1()))
                .await?;
            dev.dgc_lut_2()
                .modify(|_, w| w.value(ch.get_recommended_dgc_lut_2()))
                .await?;
            dev.dgc_lut_3()
                .modify(|_, w| w.value(ch.get_recommended_dgc_lut_3()))
                .await?;
            dev.dgc_lut_4()
                .modify(|_, w| w.value(ch.get_recommended_dgc_lut_4()))
                .await?;
            dev.dgc_lut_5()
                .modify(|_, w| w.value(ch.get_recommended_dgc_lut_5()))
                .await?;
            dev.dgc_lut_6()
                .modify(|_, w| w.value(ch.get_recommended_dgc_lut_6()))
                .await?;
            dev.dgc_cfg0().modify(|_, w| w.value(0x10000240)).await?;
            dev.dgc_cfg1().modify(|_, w| w.value(0x1b6da489)).await?;
        }

        dev.dgc_cfg().modify(|_, w| w.thr_64(0x32)).await?;
    } else {
        dev.dgc_cfg().modify(|_, w| w.rx_tune_en(0)).await?;
    }

    if config.preamble_length.get_num_of_symbols() > 64 {
        dev.dtune4().modify(|_, w| w.dtune4(0x20)).await?;
    } else {
        dev.dtune4().modify(|_, w| w.dtune4(0x14)).await?;
    }

    Ok(())
}

pub(crate) async fn otp_ldo_bias_kicks<SPI: SpiDevice>(
    dev: &mut ll::DW3000<SPI>,
) -> Result<SPI, ()> {
    let ldo_tune_l = read_otp(dev, 0x04).await?;
    let ldo_tune_h = read_otp(dev, 0x05).await?;
    let biastune_cal = (read_otp(dev, 0x0A).await? >> 0x10) & 0x1F;
    if ldo_tune_l != 0 && ldo_tune_h != 0 && biastune_cal != 0 {
        dev.otp_cfg().write(|w| w.ldo_kick(1).bias_kick(1)).await?;
        dev.bias_ctrl()
            .modify(|r, w| w.value(r.value() & 0xFFE0 | biastune_cal as u16))
            .await?
    }
    Ok(())
}

pub(crate) async fn configure_double_buffering<SPI: SpiDevice>(
    dev: &mut ll::DW3000<SPI>,
    config: &Config,
) -> Result<SPI, ()> {
    // enable diagnostics for double buffering mode
    dev.rdb_diag()
        .write(|w| {
            w.rdb_dmode(match config.enable_diagnostics {
                true => 4,
                false => 1,
            })
        })
        .await?;

    // program an indirect pointer so we can efficiently read from the second buffer set
    dev.ptr_addr_a().write(|w| w.ptra_base(0x18)).await?;
    dev.ptr_offset_a().write(|w| w.ptra_ofs(0xe8)).await?;

    Ok(())
}

async fn start_pll_calibration<SPI: SpiDevice>(
    dev: &mut ll::DW3000<SPI>,
    config: &Config,
) -> Result<SPI, ()> {
    let pmsc_state = dev.sys_state().read().await?.pmsc_state();
    if pmsc_state == 0x3 {
        dev.clk_ctrl().modify(|_, w| w.sys_clk(0x3)).await?;
        dev.seq_ctrl().modify(|_, w| w.force2init(0x1)).await?;
        dev.seq_ctrl().modify(|_, w| w.force2init(0x0)).await?;

        dev.clk_ctrl()
            .modify(|_, w| {
                w.sys_clk(0)
                    .rx_clk(0)
                    .tx_clk(0)
                    .acc_clk_en(0)
                    .cia_clk_en(0)
                    .sar_clk_en(0)
                    .acc_mclk_en(0)
            })
            .await?;
    }

    dev.rf_tx_ctrl_1().modify(|_, w| w.value(0x0E)).await?;
    dev.rf_tx_ctrl_2()
        .write(|w| w.value(config.channel.get_recommended_rf_tx_ctrl_2()))
        .await?;

    dev.pll_cfg()
        .write(|w| w.value(config.channel.get_recommended_pll_conf()))
        .await?;

    dev.ldo_rload().modify(|_, w| w.value(0x14)).await?;

    dev.pll_cal()
        .modify(|_, w| w.use_old(0x0).pll_cfg_ld(0x8))
        .await?;

    dev.sys_status().write(|w| w.cplock(1)).await?;

    dev.clk_ctrl()
        .modify(|_, w| {
            w.sys_clk(0b00)
                .rx_clk(0b00)
                .tx_clk(0b00)
                .acc_clk_en(0b0)
                .cia_clk_en(0b0)
                .sar_clk_en(0b0)
                .acc_mclk_en(0b0)
        })
        .await?;

    dev.pll_cal()
        .modify(|_, w| w.use_old(0x1).cal_en(0x1))
        .await?;

    dev.clk_ctrl().modify(|_, w| w.sys_clk(0)).await?;
    dev.seq_ctrl().modify(|_, w| w.ainit2idle(1)).await?;

    Ok(())
}

impl<SPI, O, Delay, IRQ> Device<SPI, O, IRQ, Delay, UninitializedState>
where
    SPI: SpiDevice,
    O: OutputPin,
    IRQ: Wait,
    Delay: DelayNs,
{
    pub fn new(spi: SPI, rst: O, irq: IRQ, delay: Delay) -> Self {
        let dev = ll::DW3000::new(spi);
        Self {
            config: Default::default(),
            delay,
            state: UninitializedState,
            dev,
            rst,
            irq,
        }
    }

    pub async fn init(
        mut self,
        config: Config,
    ) -> Result<SPI, Device<SPI, O, IRQ, Delay, ReadyState>> {
        let dev = &mut self.dev;

        // reset the device
        {
            self.rst.set_low().expect("failed to set rst low");
            self.delay.delay_ms(1).await;
            self.rst.set_high().expect("failed to set rst high");
        }

        // wait for the device to enter the IDLE_RC state
        //
        // XXX: Until we enter this state, we cannot use SPI. Hence the first thing we do is wait
        // for the IRQ, which is conveniently configured by default to signal only when we enter
        // IDLE_RC.
        {
            let _ = self.irq.wait_for_high().await;

            while dev.sys_status().read().await?.spirdy() != 1 {
                let _ = self.irq.wait_for_high().await;
            }
        }

        // check the device id and model
        {
            let device_id = dev.dev_id().read().await?;
            if device_id.ridtag() != 0xDECA || device_id.model() != 3 {
                return Err(Error::UnsupportedDevice);
            }
        }

        // copy pre-calibrated values from otp to their respective registers
        {
            otp_ldo_bias_kicks(dev).await?;

            let xtrim = read_otp(dev, 0x1e).await? & 0x3F;
            if xtrim != 0 {
                dev.xtal().modify(|_, w| w.value(xtrim as u8)).await?;
            } else {
                dev.xtal().modify(|_, w| w.value(0x2E)).await?;
            }

            let pll_lock_code = read_otp(dev, 0x35).await?;
            if pll_lock_code != 0 {
                dev.pll_cc().write(|w| w.value(pll_lock_code)).await?;
            }
        }

        // configure the device
        {
            dev.sys_cfg()
                .modify(|_, w| {
                    w.phr_mode(config.phr_mode as u8)
                        .phr_6m8(config.phr_rate as u8)
                        .cp_spc(0)
                        .pdoa_mode(0)
                        .cp_sdc(0)
                        .ffen(1)
                })
                .await?;

            dev.ff_cfg()
                .modify(|_, w| w.ffab(1).ffad(1).ffaa(1).ffam(1).ffib(1))
                .await?;

            dev.otp_cfg()
                .modify(|_, w| {
                    w.ops_sel(if config.preamble_length.get_num_of_symbols() >= 256 {
                        0 // long
                    } else {
                        2 // short
                    })
                    .ops_kick(1)
                })
                .await?;

            dev.dtune0()
                .modify(|_, w| w.pac(config.preamble_length.get_recommended_pac_size()))
                .await?;

            dev.tx_fctrl()
                .modify(|_, w| {
                    w.fine_plen(
                        if config.preamble_length == configs::PreambleLength::Symbols72 {
                            8
                        } else {
                            0
                        },
                    )
                })
                .await?;

            dev.dtune3().modify(|_, w| w.value(0xAF5F35CC)).await?;

            dev.chan_ctrl()
                .modify(|_, w| {
                    w.rf_chan(config.channel as u8)
                        .sfd_type(config.sfd_sequence as u8)
                        .tx_pcode(config.tx_preamble_code)
                        .rx_pcode(config.rx_preamble_code)
                })
                .await?;

            dev.tx_fctrl()
                .modify(|_, w| {
                    w.txbr(config.bit_rate as u8)
                        .txpsr(config.preamble_length as u8)
                })
                .await?;

            dev.rx_sfd_toc()
                .modify(|_, w| w.value(config.sfd_timeout))
                .await?;

            // use max power. with a low enough duty cycle, this will still be within regulatory
            // limits, but will ensure the best possible range.
            dev.tx_power().write(|w| w.value(0xffffffff)).await?;
        }

        start_pll_calibration(dev, &config).await?;

        // wait until we enter IDLE_PLL state
        {
            dev.sys_enable().write(|w| w.cplock_en(1)).await?;

            while dev.sys_status().read().await?.cplock() != 1 {
                let _ = self.irq.wait_for_high().await;
            }
        }

        tune_dgc(dev, &config).await?;
        calibrate_pgf(dev, &mut self.delay).await?;

        // set the pan id and address
        {
            dev.panadr()
                .write(|w| {
                    w.pan_id(config.pan.map(|p| p.0).unwrap_or(0xffff))
                        .short_addr(match config.address {
                            Address::Short(addr) => u16::from_be_bytes(addr),
                            _ => 0xffff,
                        })
                })
                .await?;
            if let Address::Extended(addr) = config.address {
                dev.eui()
                    .write(|w| w.value(u64::from_be_bytes(addr)))
                    .await?;
            }
        }

        // enable diagnostics if requested
        if config.enable_diagnostics {
            dev.cia_conf().modify(|_, w| w.mindiag(0)).await?;
        }

        // configure double buffering
        configure_double_buffering(dev, &config).await?;

        self.config = config;
        Ok(self.with_state(ReadyState { next_seq: 0 }))
    }
}
