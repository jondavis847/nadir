use std::any::{Any, TypeId};
use std::collections::HashMap;
use iced::widget::shader::wgpu::Buffer;

struct BufferManager {
    buffers: HashMap<TypeId, Buffer>,
}

impl BufferManager {
    fn new() -> Self {
        BufferManager {
            buffers: HashMap::new(),
        }
    }

    fn insert<T: 'static>(&mut self, buffer: Buffer) {
        let type_id = TypeId::of::<T>();
        self.buffers.insert(type_id, buffer);
    }

    fn get<T: 'static>(&self) -> Option<&Buffer> {
        let type_id = TypeId::of::<T>();
        self.buffers.get(&type_id)
    }
}