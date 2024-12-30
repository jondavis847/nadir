// running this, Vec was 4 ns, HashMap was 10 ns... we should just use HashMaps

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use nalgebra::Vector3;
use std::collections::HashMap;
use uuid::Uuid;

#[derive(Clone)]
struct A {
    name: String,
    val: Vector3<f64>,
}

fn create_vec_a() -> Vec<A> {
    (0..10)
        .map(|i| A {
            name: format!("Item {}", i),
            val: Vector3::new(i as f64, i as f64 + 1.0, i as f64 + 2.0),
        })
        .collect()
}

fn create_hashmap_a() -> HashMap<Uuid, A> {
    (0..10)
        .map(|i| {
            let id = Uuid::new_v4();
            (
                id,
                A {
                    name: format!("Item {}", i),
                    val: Vector3::new(i as f64, i as f64 + 1.0, i as f64 + 2.0),
                },
            )
        })
        .collect()
}

fn bench_vec(c: &mut Criterion) {
    let vec_a = create_vec_a();
    c.bench_function("loop over Vec<A>", |b| {
        b.iter(|| {
            for item in &vec_a {
                black_box(&item.name);
                black_box(&item.val);
            }
        });
    });
}

fn bench_hashmap(c: &mut Criterion) {
    let hashmap_a = create_hashmap_a();
    c.bench_function("loop over HashMap<Uuid, A>", |b| {
        b.iter(|| {
            for (key, item) in &hashmap_a {
                black_box(&key);
                black_box(&item.name);
                black_box(&item.val);
            }
        });
    });
}

criterion_group!(benches, bench_vec, bench_hashmap);
criterion_main!(benches);
