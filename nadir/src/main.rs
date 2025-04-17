use iced::futures::channel::mpsc;
use nadir_plots::{PlotCommand, PlotManager};
use pest_derive::Parser;
use registry::Registry;
use repl::NadirRepl;
use std::fmt::Debug;
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
    let (mut plot_command_tx, plot_command_rx) = mpsc::unbounded::<PlotCommand>();

    let plot_daemon = iced::daemon(PlotManager::title, PlotManager::update, PlotManager::view)
        .subscription(move |_state| PlotManager::subscription)
        .run_with(PlotManager::new);

    // Start the REPL on a separate thread
    let repl_thread = thread::spawn(move || {
        let repl = NadirRepl::new(registry.clone(), storage.clone(), plot_command_tx);
        repl.run();
    });
}
