use crate::sensor::measurement::Measurement;
use crate::storage::session_config::{SessionConfig, SessionType};
use crate::storage::storage_controller::StorageManager;
use crate::SendingError;

pub fn sync_from_storage<F>(
    config: &SessionConfig,
    storage: &StorageManager,
    mut send_fn: F,
) -> Result<(), SyncError>
where
    F: FnMut(&Vec<Measurement>) -> Result<(), SendingError>,
{
    let batch_size = if let SessionType::MOBILE = config.session_type {
        30
    } else {
        let free = unsafe { esp_idf_svc::sys::esp_get_free_heap_size() } as usize;
        let per_record = size_of::<Measurement>() + 4;
        let batch = (free / 2) / per_record; //we load at max half of the free RAM
        if batch < 10 {
            return Err(SyncError::NoHeapSpace);
        }
        batch.clamp(10, 500)
    };

    let iter = storage.iter_measurements().ok_or(SyncError::GetStorage)?;

    let mut measurements: Vec<Measurement> = Vec::with_capacity(batch_size);
    let mut bytes_to_remove = 0;

    for line in iter {
        if measurements.len() + line.measurements.len() > batch_size {
            break;
        }
        measurements.extend(line.measurements);
        bytes_to_remove = line.offset_from_end
    }

    match send_fn(&measurements) {
        Ok(()) => {
            if let Ok(()) = storage.remove_last(bytes_to_remove as usize) {
                Ok(())
            } else {
                Err(SyncError::RemoveStorage)
            }
        }
        Err(e) => Err(SyncError::Send(e)),
    }
}
#[derive(Debug)]
pub enum SyncError {
    GetStorage,
    RemoveStorage,
    Send(SendingError),
    NoHeapSpace,
}
