use wgpu::{
    Backends, Buffer, BufferDescriptor, BufferUsages, Device, DeviceDescriptor, Features, Instance,
    InstanceDescriptor, Limits, MemoryHints, PowerPreference, Queue, RequestAdapterOptions, Trace,
    util::{BufferInitDescriptor, DeviceExt},
};

pub struct GpuManager {
    device: Device,
    queue: Queue,
}

impl GpuManager {
    pub fn new() -> Result<Self, GpuManagerError> {
        pollster::block_on(async {
            let instance = Instance::new(&InstanceDescriptor {
                backends: Backends::all(),
                ..Default::default()
            });

            let adapter = instance
                .request_adapter(&RequestAdapterOptions {
                    power_preference: PowerPreference::HighPerformance,
                    compatible_surface: None,
                    force_fallback_adapter: false,
                })
                .await
                .map_err(GpuManagerError::AdapterNotFound)?;

            let (device, queue) = adapter
                .request_device(&DeviceDescriptor {
                    label: Some("GPU Manager Device"),
                    required_features: Features::empty(),
                    required_limits: Limits::default(),
                    memory_hints: MemoryHints::default(),
                    trace: Trace::Off,
                })
                .await
                .map_err(GpuManagerError::DeviceRequest)?;

            Ok(Self { device, queue })
        })
    }

    /// Create a buffer that can be written to via mapping
    pub fn create_input_buffer<T: bytemuck::Pod>(&self, label: &str, data: &[T]) -> Buffer {
        let bytes = bytemuck::cast_slice(data);
        self.device
            .create_buffer_init(&BufferInitDescriptor {
                label: Some(label),
                contents: bytes,
                usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
            })
    }

    /// Write to buffer using mapping (for frequent updates)
    pub fn write_input_buffer<T: bytemuck::Pod>(
        &self,
        buffer: &Buffer,
        data: &[T],
    ) -> Result<(), GpuManagerError> {
        let buffer_slice = buffer.slice(..);

        let done = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let done_clone = done.clone();

        buffer_slice.map_async(
            wgpu::MapMode::Write,
            move |_| {
                done_clone.store(
                    true,
                    std::sync::atomic::Ordering::Release,
                );
            },
        );

        // Poll until mapping is complete
        while !done.load(std::sync::atomic::Ordering::Acquire) {
            self.device
                .poll(wgpu::PollType::Wait)?;
            std::thread::yield_now();
        }

        let mut mapped_data = buffer_slice.get_mapped_range_mut();
        let gpu_data: &mut [T] = bytemuck::cast_slice_mut(&mut mapped_data);
        let copy_len = gpu_data
            .len()
            .min(data.len());
        gpu_data[..copy_len].copy_from_slice(&data[..copy_len]);

        drop(mapped_data); // Unmap the buffer
        buffer.unmap();

        Ok(())
    }

    // Creates a staging buffer on the GPU, which the CPU can retrieve data from
    pub fn create_output_buffer(&self, label: &str, size: u64) -> Buffer {
        self.device
            .create_buffer(&BufferDescriptor {
                label: Some(label),
                size,
                usage: BufferUsages::COPY_DST | BufferUsages::MAP_READ,
                mapped_at_creation: false,
            })
    }

    pub fn read_output_buffer<T: bytemuck::Pod>(
        &self,
        buffer: &Buffer,
        output: &mut [T],
    ) -> Result<usize, GpuManagerError> {
        if output.is_empty() {
            return Err(GpuManagerError::InvalidBufferSize("Output buffer is empty".to_string()));
        }

        let buffer_slice = buffer.slice(..);

        // Simple flag to track completion
        let done = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let done_clone = done.clone();

        buffer_slice.map_async(
            wgpu::MapMode::Read,
            move |_| {
                done_clone.store(
                    true,
                    std::sync::atomic::Ordering::Release,
                );
            },
        );

        // Poll until mapping is complete
        while !done.load(std::sync::atomic::Ordering::Acquire) {
            self.device
                .poll(wgpu::PollType::Wait)?;
            std::thread::yield_now();
        }

        let mapped_data = buffer_slice.get_mapped_range();
        let gpu_data: &[T] = bytemuck::cast_slice(&mapped_data);
        let copy_len = output
            .len()
            .min(gpu_data.len());
        output[..copy_len].copy_from_slice(&gpu_data[..copy_len]);

        Ok(copy_len)
    }
}

#[derive(Debug, thiserror::Error)]
pub enum GpuManagerError {
    #[error("GPU Adapter adapter error: {0}")]
    AdapterNotFound(#[from] wgpu::RequestAdapterError),
    #[error("GPU Device request error: {0}")]
    DeviceRequest(#[from] wgpu::RequestDeviceError),
    #[error("{0}")]
    InvalidBufferSize(String),
    #[error("Buffer read failed")]
    BufferRead,
    #[error("GPU poll error: {0}")]
    PollError(#[from] wgpu::PollError),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_input_buffer() {
        let manager = GpuManager::new().unwrap();

        // Create buffer with test data
        let test_data = [1.0f32, 2.0, 3.0, 4.0];
        let buffer = manager.create_input_buffer("test_data_input", &test_data);

        // Verify buffer was created with correct size
        assert_eq!(
            buffer.size(),
            (test_data.len() * std::mem::size_of::<f32>()) as u64
        );
    }

    #[test]
    fn test_create_output_buffer() {
        let manager = GpuManager::new().unwrap();

        // Create output buffer with specific size
        let buffer_size = 128;
        let buffer = manager.create_output_buffer(
            "test_data_output",
            buffer_size,
        );

        // Verify buffer was created with correct size and usage
        assert_eq!(buffer.size(), buffer_size);
    }

    #[test]
    fn test_read_buffer_empty_output() {
        let manager = GpuManager::new().unwrap();
        let buffer = manager.create_output_buffer("test_output_data", 16);
        let mut empty_output: [f32; 0] = [];

        // Should fail with empty output buffer
        let result = manager.read_output_buffer(&buffer, &mut empty_output);
        assert!(result.is_err());

        if let Err(GpuManagerError::InvalidBufferSize(msg)) = result {
            assert!(
                msg.contains("empty"),
                "Error should mention empty buffer"
            );
        } else {
            panic!(
                "Expected InvalidBufferSize error, got: {:?}",
                result
            );
        }
    }

    #[test]
    fn test_read_buffer_round_trip() {
        let manager = GpuManager::new().unwrap();

        // Create source data
        let source_data = [1.0f32, 2.0, 3.0, 4.0];

        // Create source buffer
        let source_buffer = manager.create_input_buffer("source_buffer", &source_data);

        // Create destination buffer
        let dest_buffer = manager.create_output_buffer(
            "dest_buffer",
            source_buffer.size(),
        );

        // Create encoder and copy from source to destination
        let mut encoder = manager
            .device
            .create_command_encoder(
                &wgpu::CommandEncoderDescriptor { label: Some("test_encoder") },
            );

        encoder.copy_buffer_to_buffer(
            &source_buffer,
            0,
            &dest_buffer,
            0,
            source_buffer.size(),
        );

        // Submit the copy command
        manager
            .queue
            .submit(std::iter::once(
                encoder.finish(),
            ));

        // Read back the data
        let mut result_data = [0.0f32; 4];
        let copied = manager
            .read_output_buffer(&dest_buffer, &mut result_data)
            .expect("Buffer read failed");

        // Verify data
        assert_eq!(
            copied,
            source_data.len(),
            "Should have copied all elements"
        );
        assert_eq!(
            result_data, source_data,
            "Data read from GPU should match original data"
        );
    }

    #[test]
    fn test_read_buffer_partial_copy() {
        let manager = GpuManager::new().unwrap();

        // Create source data
        let source_data = [1.0f32, 2.0, 3.0, 4.0, 5.0, 6.0];

        // Create source buffer
        let source_buffer = manager.create_input_buffer("source_buffer", &source_data);

        // Create destination buffer
        let dest_buffer = manager.create_output_buffer(
            "dest_buffer",
            source_buffer.size(),
        );

        // Create encoder and copy from source to destination
        let mut encoder = manager
            .device
            .create_command_encoder(
                &wgpu::CommandEncoderDescriptor { label: Some("test_encoder") },
            );

        encoder.copy_buffer_to_buffer(
            &source_buffer,
            0,
            &dest_buffer,
            0,
            source_buffer.size(),
        );

        // Submit the copy command
        manager
            .queue
            .submit(std::iter::once(
                encoder.finish(),
            ));

        // Create an output buffer smaller than the source
        let mut result_data = [0.0f32; 3];
        let copied = manager
            .read_output_buffer(&dest_buffer, &mut result_data)
            .expect("Buffer read failed");

        // Verify data
        assert_eq!(
            copied,
            result_data.len(),
            "Should have copied output.len() elements"
        );
        assert_eq!(
            result_data,
            source_data[..3],
            "Data read should match first 3 elements"
        );
    }

    #[test]
    fn test_read_buffer_larger_output() {
        let manager = GpuManager::new().unwrap();

        // Create source data
        let source_data = [1.0f32, 2.0];

        // Create source buffer
        let source_buffer = manager.create_input_buffer("source_buffer", &source_data);

        // Create destination buffer
        let dest_buffer = manager.create_output_buffer(
            "dest_buffer",
            source_buffer.size(),
        );

        // Create encoder and copy from source to destination
        let mut encoder = manager
            .device
            .create_command_encoder(
                &wgpu::CommandEncoderDescriptor { label: Some("test_encoder") },
            );

        encoder.copy_buffer_to_buffer(
            &source_buffer,
            0,
            &dest_buffer,
            0,
            source_buffer.size(),
        );

        // Submit the copy command
        manager
            .queue
            .submit(std::iter::once(
                encoder.finish(),
            ));

        // Output buffer larger than source
        let mut result_data = [0.0f32; 4];
        let copied = manager
            .read_output_buffer(&dest_buffer, &mut result_data)
            .expect("Buffer read failed");

        // Verify data
        assert_eq!(
            copied,
            source_data.len(),
            "Should have copied source.len() elements"
        );
        assert_eq!(
            result_data[..copied],
            source_data,
            "Copied data should match source"
        );
        assert_eq!(
            result_data[copied..],
            [0.0, 0.0],
            "Rest of output should be untouched"
        );
    }

    #[test]
    fn test_read_buffer_different_types() {
        let manager = GpuManager::new().unwrap();

        // Create source data as integers
        let source_data = [1u32, 2, 3, 4];

        // Create source buffer
        let source_buffer = manager.create_input_buffer("source_buffer", &source_data);

        // Create destination buffer
        let dest_buffer = manager.create_output_buffer(
            "dest_buffer",
            source_buffer.size(),
        );

        // Create encoder and copy from source to destination
        let mut encoder = manager
            .device
            .create_command_encoder(
                &wgpu::CommandEncoderDescriptor { label: Some("test_encoder") },
            );

        encoder.copy_buffer_to_buffer(
            &source_buffer,
            0,
            &dest_buffer,
            0,
            source_buffer.size(),
        );

        // Submit the copy command
        manager
            .queue
            .submit(std::iter::once(
                encoder.finish(),
            ));

        // Read as unsigned integers
        let mut result_data = [0u32; 4];
        let copied = manager
            .read_output_buffer(&dest_buffer, &mut result_data)
            .expect("Buffer read failed");

        // Verify data
        assert_eq!(copied, source_data.len());
        assert_eq!(result_data, source_data);
    }

    #[test]
    fn test_read_buffer_complex_struct() {
        #[repr(C)]
        #[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable, PartialEq)]
        struct TestVertex {
            position: [f32; 3],
            color: [f32; 4],
        }

        let manager = GpuManager::new().unwrap();

        // Create source data
        let source_data = [
            TestVertex { position: [1.0, 2.0, 3.0], color: [1.0, 0.0, 0.0, 1.0] },
            TestVertex { position: [4.0, 5.0, 6.0], color: [0.0, 1.0, 0.0, 1.0] },
        ];

        // Create source buffer
        let source_buffer = manager.create_input_buffer("source_buffer", &source_data);

        // Create destination buffer
        let dest_buffer = manager.create_output_buffer(
            "dest_buffer",
            source_buffer.size(),
        );

        // Create encoder and copy from source to destination
        let mut encoder = manager
            .device
            .create_command_encoder(
                &wgpu::CommandEncoderDescriptor { label: Some("test_encoder") },
            );

        encoder.copy_buffer_to_buffer(
            &source_buffer,
            0,
            &dest_buffer,
            0,
            source_buffer.size(),
        );

        // Submit the copy command
        manager
            .queue
            .submit(std::iter::once(
                encoder.finish(),
            ));

        // Read back the data
        let mut result_data =
            [TestVertex { position: [0.0, 0.0, 0.0], color: [0.0, 0.0, 0.0, 0.0] }; 2];

        let copied = manager
            .read_output_buffer(&dest_buffer, &mut result_data)
            .expect("Buffer read failed");

        // Verify data
        assert_eq!(copied, source_data.len());
        assert_eq!(result_data, source_data);
    }

    #[test]
    fn test_concurrent_buffer_operations() {
        let manager = GpuManager::new().unwrap();

        // Create multiple buffers
        let data1 = [1.0f32, 2.0, 3.0, 4.0];
        let data2 = [5.0f32, 6.0, 7.0, 8.0];

        let buffer1 = manager.create_input_buffer("data1_input", &data1);

        let buffer2 = manager.create_input_buffer("data2_input", &data2);

        let output1 = manager.create_output_buffer("data1_output", buffer1.size());
        let output2 = manager.create_output_buffer("data2_output", buffer2.size());

        // Create encoder and submit multiple operations
        let mut encoder = manager
            .device
            .create_command_encoder(
                &wgpu::CommandEncoderDescriptor { label: Some("test_encoder") },
            );

        encoder.copy_buffer_to_buffer(
            &buffer1,
            0,
            &output1,
            0,
            buffer1.size(),
        );
        encoder.copy_buffer_to_buffer(
            &buffer2,
            0,
            &output2,
            0,
            buffer2.size(),
        );

        manager
            .queue
            .submit(std::iter::once(
                encoder.finish(),
            ));

        // Read both buffers
        let mut result1 = [0.0f32; 4];
        let mut result2 = [0.0f32; 4];

        let copied1 = manager
            .read_output_buffer(&output1, &mut result1)
            .expect("Buffer read failed");
        let copied2 = manager
            .read_output_buffer(&output2, &mut result2)
            .expect("Buffer read failed");

        // Verify data
        assert_eq!(copied1, data1.len());
        assert_eq!(copied2, data2.len());
        assert_eq!(result1, data1);
        assert_eq!(result2, data2);
    }
}
