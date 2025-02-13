use std::any::Any;

pub trait CloneAny: Any {
    fn clone_any(&self) -> Box<dyn CloneAny>;
    fn type_name(&self) -> &'static str;
}

impl<T> CloneAny for T
where
    T: Any + Clone + 'static,
{
    fn clone_any(&self) -> Box<dyn CloneAny> {
        Box::new(self.clone())
    }

    fn type_name(&self) -> &'static str {
        std::any::type_name::<T>()
    }
}

impl Clone for Box<dyn CloneAny> {
    fn clone(&self) -> Box<dyn CloneAny> {
        self.clone_any()
    }
}
