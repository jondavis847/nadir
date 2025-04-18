use iced::daemon;
use iced::futures::channel::mpsc::{self, Sender};
use pest_derive::Parser;
use plot_manager::PlotManager;
use registry::Registry;
use repl::NadirRepl;
use std::fmt::Debug;
use std::sync::{Arc, Mutex};
use std::thread;
mod helper;
mod plot_manager;
mod registry;
mod repl;
mod storage;
mod value;
use storage::Storage;

#[derive(Parser)]
#[grammar = "main.pest"] // relative path to your .pest file
struct NadirParser;

#[derive(Debug)]
enum ReplToDaemon {}

#[derive(Debug)]
enum DaemonToRepl {
    ReplToSubscriptionTx(Sender<ReplToSubscription>),
}

#[derive(Debug)]
enum ReplToSubscription {
    CloseAllFigures,
    NewFigure,
    ReplClosed,
}

#[derive(Debug)]
enum DaemonToSubscription {}

fn main() {
    // create 2 channels that allow the iced daemon and repl thread to communicate
    let (repl_to_daemon_tx, repl_to_daemon_rx) = mpsc::channel::<ReplToDaemon>(10);
    let (daemon_to_repl_tx, daemon_to_repl_rx) = mpsc::channel::<DaemonToRepl>(10);

    let registry = Arc::new(Mutex::new(Registry::new()));
    let storage = Arc::new(Mutex::new(Storage::default()));

    // Start the REPL on a separate thread
    thread::spawn(move || {
        let mut repl = NadirRepl::new(registry.clone(), storage.clone());
        // TODO: make the daemon only start when necessary on command
        repl.connect_plot_daemon(repl_to_daemon_tx, daemon_to_repl_rx);
        if let Err(e) = repl.run() {
            eprintln!("{:?}", e)
        };
    });

    if let Err(e) = daemon(PlotManager::title, PlotManager::update, PlotManager::view)
        .subscription(PlotManager::subscription)
        .run_with(|| PlotManager::new(daemon_to_repl_tx, repl_to_daemon_rx))
    {
        eprintln!("{:?}", e)
    };
}
