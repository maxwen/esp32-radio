use alloc::vec::Vec;

use embedded_sdmmc::{BlockDevice, DirEntry, Error, RawDirectory, RawFile, RawVolume, ShortFileName, VolumeIdx};
use embedded_sdmmc::filesystem::ToShortFileName;

pub struct FakeTimesource();

impl embedded_sdmmc::TimeSource for FakeTimesource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

pub struct SdcardManager<D: BlockDevice> {
    volume_manager: embedded_sdmmc::VolumeManager<D, FakeTimesource>,
    root_volume: Option<RawVolume>,
    root_dir: Option<RawDirectory>,
}

impl<D: embedded_sdmmc::BlockDevice> SdcardManager<D> {
    pub fn new(sdcard: D) -> Self {
        SdcardManager {
            volume_manager: embedded_sdmmc::VolumeManager::new(sdcard, FakeTimesource()),
            root_volume: None,
            root_dir: None,
        }
    }

    pub fn get_root_dir_entries(&mut self, file_list: &mut Vec<DirEntry>) -> Result<(), Error<D::Error>> {
        if self.root_volume.is_some() && self.root_dir.is_some() {
            self.volume_manager.iterate_dir(self.root_dir.unwrap(), |d| {
                let dir = d.clone();
                if !dir.attributes.is_directory() {
                    file_list.push(dir);
                }
            }).unwrap();
        }

        Ok(())
    }

    pub fn load_file_into_buffer(&mut self, name: ShortFileName, buffer: &mut [u8]) -> Result<(), Error<D::Error>> {
        if self.root_volume.is_some() && self.root_dir.is_some() {
            let file = self.volume_manager.open_file_in_dir(self.root_dir.unwrap(), name, embedded_sdmmc::Mode::ReadOnly)?;
            self.volume_manager.read(file, buffer)?;
            self.volume_manager.close_file(file)?;
        }
        Ok(())
    }

    pub fn open_root_dir(&mut self) -> Result<(), Error<D::Error>> {
        let volume0 = self.volume_manager.open_raw_volume(VolumeIdx(0))?;
        self.root_volume = Some(volume0);
        let root_dir = self.volume_manager.open_root_dir(volume0)?;
        self.root_dir = Some(root_dir);
        Ok(())
    }

    pub fn close_root_dir(&mut self) -> Result<(), Error<D::Error>> {
        if self.root_dir.is_some() {
            self.volume_manager.close_dir(self.root_dir.unwrap())?;
        }
        if self.root_volume.is_some() {
            self.volume_manager.close_volume(self.root_volume.unwrap())?;
        }
        Ok(())
    }

    pub fn get_subdir_entries<N>(&mut self, dir: N, file_list: &mut Vec<DirEntry>) -> Result<(), Error<D::Error>> where N: ToShortFileName {
        if self.root_volume.is_some() && self.root_dir.is_some() {
            let sub_dir = self.volume_manager.open_dir(self.root_dir.unwrap(), dir)?;
            self.volume_manager.iterate_dir(sub_dir, |d| {
                let dir = d.clone();
                if !dir.attributes.is_directory() {
                    file_list.push(dir);
                }
            }).unwrap();
            self.volume_manager.close_dir(sub_dir)?;
        }

        Ok(())
    }

    pub fn load_subdir_file_into_buffer<N>(&mut self, dir: N, name: N, buffer: &mut [u8]) -> Result<(), Error<D::Error>> where N: ToShortFileName {
        if self.root_volume.is_some() && self.root_dir.is_some() {
            let sub_dir = self.volume_manager.open_dir(self.root_dir.unwrap(), dir)?;
            let file = self.volume_manager.open_file_in_dir(sub_dir, name, embedded_sdmmc::Mode::ReadOnly)?;
            self.volume_manager.read(file, buffer)?;
            self.volume_manager.close_file(file)?;
            self.volume_manager.close_dir(sub_dir)?;
        }
        Ok(())
    }
    pub fn write_file_in_root_dir_from_buffer(&mut self, file: RawFile, buffer: &[u8]) -> Result<(), Error<D::Error>> {
        self.volume_manager.write(file, buffer)
    }

    pub fn open_file_in_root_dir_for_writing<N>(&mut self, name: N) -> Result<RawFile, Error<D::Error>> where N: ToShortFileName {
        if self.root_volume.is_some() && self.root_dir.is_some() {
            return self.volume_manager.open_file_in_dir(self.root_dir.unwrap(), name, embedded_sdmmc::Mode::ReadWriteCreateOrTruncate);
        }
        Err(Error::NotFound)
    }
    pub fn close_file(&mut self, file: RawFile) -> Result<(), Error<D::Error>> {
        self.volume_manager.close_file(file)
    }

    pub fn delete_file_in_root_dir<N>(&mut self, file: N) -> Result<(), Error<D::Error>> where N: ToShortFileName {
        if self.root_volume.is_some() && self.root_dir.is_some() {
            return match self.volume_manager.delete_file_in_dir(self.root_dir.unwrap(), file) {
                _ => Ok(()),
            }
        }
        Err(Error::NotFound)
    }
}