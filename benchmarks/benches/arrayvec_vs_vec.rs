use arrayvec::ArrayVec;
use criterion::{criterion_group, criterion_main, Criterion};

use mass_properties::MassProperties;

#[allow(dead_code)]
#[derive(Copy, Clone)]
struct BodyNoString {
    pub mass_properties: MassProperties,
}

#[allow(dead_code)]
#[derive(Clone)]
struct BodyString {
    name: String,
    mass_properties: MassProperties,
}

fn clone_arrayvec(array_vec: ArrayVec<BodyNoString, 100>) -> ArrayVec<BodyNoString, 100> {
    //array_vec.clone()
    array_vec
}

fn clone_arrayvec_5(array_vec: ArrayVec<BodyNoString, 5>) -> ArrayVec<BodyNoString, 5> {
    //array_vec.clone()
    array_vec
}

fn clone_array_5(array: [BodyNoString; 5]) -> [BodyNoString; 5] {
    //array_vec.clone()
    array
}

fn clone_vec(vec: Vec<BodyNoString>) -> Vec<BodyNoString> {
    vec.clone()
}

fn criterion_benchmark(c: &mut Criterion) {
    let mp = MassProperties::default();
    let body = BodyNoString {
        mass_properties: mp,
    };

    let mut array_vec_3: ArrayVec<BodyNoString, 100> = ArrayVec::new();
    let mut vec_3 = Vec::new();
    let mut array_vec_3_5: ArrayVec<BodyNoString, 5> = ArrayVec::new();
    let array_3_5 = [body, body, body, body, body];
    for _i in 0..3 {
        array_vec_3.push(body.clone());
        vec_3.push(body.clone());
        array_vec_3_5.push(body.clone());
    }

    let mut array_vec_50: ArrayVec<BodyNoString, 100> = ArrayVec::new();
    let mut vec_50 = Vec::new();
    for _i in 0..50 {
        array_vec_50.push(body.clone());
        vec_50.push(body.clone());
    }

    let mut array_vec_100: ArrayVec<BodyNoString, 100> = ArrayVec::new();
    let mut vec_100 = Vec::new();
    for _i in 0..100 {
        array_vec_100.push(body.clone());
        vec_100.push(body.clone());
    }

    c.bench_function("clone_arrayvec_3", |b| {
        b.iter(|| clone_arrayvec(array_vec_3.clone()))
    });
    c.bench_function("clone_vec_3", |b| b.iter(|| clone_vec(vec_3.clone())));
    c.bench_function("clone_arrayvec_3_5", |b| {
        b.iter(|| clone_arrayvec_5(array_vec_3_5.clone()))
    });
    c.bench_function("clone_array_3_5", |b| {
        b.iter(|| clone_array_5(array_3_5.clone()))
    });

    c.bench_function("clone_arrayvec_50", |b| {
        b.iter(|| clone_arrayvec(array_vec_50.clone()))
    });
    c.bench_function("clone_vec_50", |b| b.iter(|| clone_vec(vec_50.clone())));

    c.bench_function("clone_arrayvec_100", |b| {
        b.iter(|| clone_arrayvec(array_vec_100.clone()))
    });
    c.bench_function("clone_vec_100", |b| b.iter(|| clone_vec(vec_100.clone())));
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
