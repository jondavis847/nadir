use nadir_plots::{PlotCommand, PlotManager};
use pest_derive::Parser;
use registry::Registry;
use repl::NadirRepl;
use std::fmt::Debug;
use std::sync::{Arc, Mutex, mpsc};
use std::thread;
mod helper;
mod registry;
mod repl;
mod storage;
mod value;
use storage::Storage;

#[derive(Parser)]
#[grammar = "main.pest"] // relative path to your .pest file
struct NadirParser;

fn main() {
    let registry = Arc::new(Mutex::new(Registry::default()));
    let storage = Arc::new(Mutex::new(Storage::default()));

    // set up the plot interface
    // Create a channel for REPL to send commands to the plot manager
    let (command_tx, command_rx) = mpsc::channel::<PlotCommand>();

    // Wrap receiver in Arc<Mutex> so it can be accessed from the subscription
    let shared_receiver = Arc::new(Mutex::new(command_rx));
    let repl_command_tx = command_tx.clone();

    let plot_daemon = iced::daemon(PlotManager::title, PlotManager::update, PlotManager::view)
        .subscription(move |_state| PlotManager::subscription(_state, shared_receiver.clone()))
        .run_with(PlotManager::new);

    // Start the REPL on a separate thread
    let repl_thread = thread::spawn(move || {
        let repl = NadirRepl::new(registry.clone(), storage.clone());
        repl.run();
    });
}
