use crate::{
    uninitialized::{calibrate_pgf, configure_double_buffering, otp_ldo_bias_kicks, tune_dgc},
    Device, Error, ReadyState, Result,
};

use dw3000_ng::ll;
use embedded_hal_async::{
    delay::DelayNs,
    digital::Wait,
    spi::{self, SpiDevice},
};
use smoltcp::wire::ieee802154::Address;

pub struct AsleepState {
    pub(crate) ready_state: ReadyState,
}

impl<SPI, O, IRQ, Delay> Device<SPI, O, IRQ, Delay, AsleepState>
where
    SPI: SpiDevice,
    IRQ: Wait,
    Delay: DelayNs,
{
    pub async fn wake(mut self) -> Result<SPI, Device<SPI, O, IRQ, Delay, ReadyState>> {
        // wake the device up via the chip select pin
        self.dev
            .spi
            .transaction(&mut [spi::Operation::DelayNs(800_000)])
            .await
            .map_err(|e| Error::Device(ll::Error::Transfer(e)))?;

        // wait for the device to enter the IDLE_RC state and accept spi transactions
        {
            self.wait_for_irq_with_timeout_us(100_000).await?;

            if self.dev.sys_status().read().await?.spirdy() != 1 {
                return Err(Error::Unknown);
            }
        }

        // reload the things that weren't saved by the aon memory
        {
            otp_ldo_bias_kicks(&mut self.dev).await?;
        }

        // transition into the IDLE_PLL state
        {
            self.dev.sys_enable().write(|w| w.cplock_en(1)).await?;
            self.dev.seq_ctrl().modify(|_, w| w.ainit2idle(1)).await?;

            loop {
                let status = self.dev.sys_status().read().await?;
                if status.cplock() == 1 {
                    break;
                }
                self.wait_for_irq_with_timeout_us(10_000).await?;
            }
        }

        // reconfigure a few more things...
        {
            tune_dgc(&mut self.dev, &self.config).await?;
            calibrate_pgf(&mut self.dev, &mut self.delay).await?;
            configure_double_buffering(&mut self.dev, &self.config).await?;
            if let Address::Extended(addr) = self.config.address {
                self.dev
                    .eui()
                    .write(|w| w.value(u64::from_be_bytes(addr)))
                    .await?;
            }
        }

        let ready_state = self.state.ready_state.clone();
        Ok(self.with_state(ready_state))
    }
}
