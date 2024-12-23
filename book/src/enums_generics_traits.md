# Enums vs. Generics vs. Trait Objects
I spent a lot of time tryign to find the best answer for my program regarding flexibility in designing and making collections of objects. I'm not sure I arrived at the best and most idiomatic method, but these were the design considerations.

## Enums
Enums end up seeming like the best answer. They retain the strong typing and compile time checks that Rust is known for, and avoids heap allocations of Box, but most importantly for my use case, allow me to make collections of different subtypes. i.e. I can collect all Sensors as an array of Sensors, but have different submodels of Sensors (StarTracker, RateGyro, etc.) in the collection. This is required for multibody dynamics algorithms. 

Another thing that is really nice about enums is I can define associated types for my traits which you can't do for trait objects.

The only con with enums is the boiler plate. I end up with lots of just extra code that looks like this:

```rust
pub enum SensorModel {
    RateGyro(RateGyro),
    StarTracker(StarTracker),    
}

pub trait SensorModelTrait: MultibodyResult {
    fn update(&mut self, connection: &BodyConnection);
}

impl SensorModelTrait for SensorModel {
    fn update(&mut self, connection: &BodyConnection) {
        match self {
            SensorModel::RateGyro(sensor) => sensor.update(connection),
            SensorModel::StarTracker(sensor) => sensor.update(connection),
        }
    }
}

impl MultibodyResult for SensorModel {
    fn initialize_result(&self, writer: &mut Writer<BufWriter<File>>) {
        match self {
            SensorModel::RateGyro(sensor) => sensor.initialize_result(writer),
            SensorModel::StarTracker(sensor) => sensor.initialize_result(writer),
        }
    }

    fn write_result_file(&self, writer: &mut Writer<BufWriter<File>>) {
        match self {
            SensorModel::RateGyro(sensor) => sensor.write_result_file(writer),
            SensorModel::StarTracker(sensor) => sensor.write_result_file(writer),
        }
    }
}
```

It is quite obnoxious, but I do think this is the technically correct answer. 
Another con might be extensibility. If a user wants to add their own sensor in a fork, they would have to go update all of this boilerplate to add their new type.

Update: The crate amabassador seems to solve the boilerplate issue.

## Generics
Generics are a great option but don't allow me to directly create heterogenous collections of say 
```rust 
[Sensor<RateGyro>, Sensor<StarTracker>]
```
I would instead need to either use trait objects, or create some new struct that holds all of the types. For example:

```rust 
struct SensorSystem {
    rate_gyro: Sensor<RateGyro>,
    star_tracker: Sensor<StarTracker>,
}
```


## Trait Objects

Trait Objects seem like a great option to reduce boilerplate, allow flexibility and extensibility, and I can create collections of them. There are techinically performance impacts due to heap allocation and dynamic dispatch. Thus far, these do not seem to significantly impact sim performance since all heap allocations are preallocated. The main problem with trait objects is downcasting to the type I need. For example, when I want to connect sensor to software, star tracker software is expecting a &StarTracker, not &Box<dyn SensorModel> . It seems there are ways to downcast Box<dyn SensorModel> to a StarTracker using std::any::Any, but I don't love users having to learn how to do that to get their software to work.
