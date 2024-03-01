use core::fmt::Debug;

use embedded_hal_async::i2c::I2c;

pub struct Tsc2007<I2C> {
    i2c: I2C,
    buf: [u8; 2],
    cmd: [u8; 1],
}

pub const TSC2007_ADDR: u8 = 0x48;
const TSC2007_MEASURE_TEMP0: u8 = 0;
const TSC2007_MEASURE_AUX: u8 = 2;
const TSC2007_MEASURE_TEMP1: u8 = 4;
const TSC2007_ACTIVATE_X: u8 = 8;
const TSC2007_ACTIVATE_Y: u8 = 9;
const TSC2007_ACTIVATE_YPLUS_X: u8 = 10;
const TSC2007_SETUP_COMMAND: u8 = 11;
const TSC2007_MEASURE_X: u8 = 12;
const TSC2007_MEASURE_Y: u8 = 13;
const TSC2007_MEASURE_Z1: u8 = 14;
const TSC2007_MEASURE_Z2: u8 = 15;

const TSC2007_POWERDOWN_IRQON: u8 = 0;
const TSC2007_ADON_IRQOFF: u8 = 1;
const TSC2007_ADOFF_IRQON: u8 = 2;

const TSC2007_ADC_12BIT: u8 = 0;
const TSC2007_ADC_8BIT: u8 = 1;

pub(crate) const TS_MINX: u16 = 550;
pub(crate) const TS_MINY: u16 = 350;
pub(crate) const TS_MAXX: u16 = 3600;
pub(crate) const TS_MAXY: u16 = 3700;
pub(crate) const TS_MIN_PRESSURE: u16 = 100;

impl<I2C: I2c> Tsc2007<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Tsc2007 {
            i2c,
            buf: [0u8; 2],
            cmd: [0u8; 1],
        }
    }

    async fn command(&mut self, function: u8, power: u8, resolution: u8) -> Result<u16, I2C::Error> {
        self.cmd[0] = (function & 0x0F) << 4;
        self.cmd[0] |= (power & 0x03) << 2;
        self.cmd[0] |= (resolution & 0x01) << 1;

        return match self.i2c.write_read(TSC2007_ADDR, &self.cmd, &mut self.buf).await {
            Ok(()) => {
                let result = (self.buf[0] as u16) << 4 | (self.buf[1] as u16) >> 4;
                Ok(result)
            }
            Err(e) => {
                Err(e)
            }
        };
    }

    pub async fn touched(&mut self) -> bool {
        if let Ok(point) = self.touch().await {
            return point.2 > TS_MIN_PRESSURE;
        }
        false
    }

    pub async fn touch(&mut self) -> Result<(u16, u16, u16), I2C::Error> {
        let x = self.command(TSC2007_MEASURE_X, TSC2007_ADON_IRQOFF, TSC2007_ADC_12BIT).await?;
        let y = self.command(TSC2007_MEASURE_Y, TSC2007_ADON_IRQOFF, TSC2007_ADC_12BIT).await?;
        let z = self.command(TSC2007_MEASURE_Z1, TSC2007_ADON_IRQOFF, TSC2007_ADC_12BIT).await?;
        self.command(TSC2007_MEASURE_TEMP0, TSC2007_POWERDOWN_IRQON, TSC2007_ADC_12BIT).await?;
        Ok((x, y, z))
    }
}