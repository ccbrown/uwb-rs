use crate::{Device, Error, ReadyState, Result};

use dw3000_ng::ll;
use embedded_hal_async::{
    delay::DelayNs,
    digital::Wait,
    spi::{self, SpiDevice},
};

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
        let dev = &mut self.dev;

        // wake the device up via the chip select pin
        dev.spi
            .transaction(&mut [spi::Operation::DelayNs(800_000)])
            .await
            .map_err(|e| Error::Device(ll::Error::Transfer(e)))?;

        // wait for the device to enter the IDLE_RC state and accept spi transactions
        {
            let _ = self.irq.wait_for_high().await;

            while dev.sys_status().read().await?.spirdy() != 1 {
                let _ = self.irq.wait_for_high().await;
            }
        }

        // wait until we enter IDLE_PLL state
        {
            dev.sys_enable().write(|w| w.cplock_en(1)).await?;

            while dev.sys_status().read().await?.cplock() != 0 {
                let _ = self.irq.wait_for_high().await;
            }
        }

        let ready_state = self.state.ready_state.clone();
        Ok(self.with_state(ready_state))
    }
}
