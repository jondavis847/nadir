use criterion::{black_box, criterion_group, criterion_main, Criterion};

#[derive(Debug, Default, Clone, Copy)]
pub struct RevoluteState {
    theta: f64,
    omega: f64,
    q_ddot: f64,
}

fn copy_array() {
    let states: [RevoluteState; 5] = [RevoluteState::default(); 5];
    let _copied_states = black_box(states);
}

fn clone_array() {
    let states: [RevoluteState; 5] = [RevoluteState::default(); 5];
    let _cloned_states = black_box(states.clone());
}

fn copy_vec() {
    let states: Vec<RevoluteState> = vec![RevoluteState::default(); 5];
    let _copied_states = black_box(states);
}

fn clone_vec() {
    let states: Vec<RevoluteState> = vec![RevoluteState::default(); 5];
    let _cloned_states = black_box(states.clone());
}

fn benchmark(c: &mut Criterion) {
    c.bench_function("copy_array", |b| b.iter(|| copy_array()));
    c.bench_function("clone_array", |b| b.iter(|| clone_array()));
    c.bench_function("copy_vec", |b| b.iter(|| copy_vec()));
    c.bench_function("clone_vec", |b| b.iter(|| clone_vec()));
}

criterion_group!(benches, benchmark);
criterion_main!(benches);
