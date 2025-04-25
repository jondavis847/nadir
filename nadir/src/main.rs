use iced::daemon;
use iced::futures::channel::mpsc::{self, Sender};
use iced::window::Id;
use pest_derive::Parser;
use plotting::figure::Figure;
use registry::Registry;
use repl::NadirRepl;
use std::fmt::Debug;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::thread;
use window_manager::WindowManager;
mod animation;
mod helper;
mod plotting;
mod registry;
mod repl;
mod storage;
mod value;
mod window_manager;
use storage::Storage;

#[derive(Parser)]
#[grammar = "main.pest"] // relative path to your .pest file
struct NadirParser;

#[derive(Debug)]
enum ReplToDaemon {}

#[derive(Debug)]
enum DaemonToRepl {
    PlotReady(Arc<Mutex<Figure>>),
    ReplToSubscriptionTx(Sender<ReplToSubscription>),
}

#[derive(Debug)]
enum ReplToSubscription {
    Animate(PathBuf),
    ClearCache(Id),
    CloseAllFigures,
    NewFigure(Arc<Mutex<Figure>>),
    ReplClosed,
}

fn main() {
    // create 2 channels that allow the iced daemon and repl thread to communicate
    let (repl_to_daemon_tx, _repl_to_daemon_rx) = mpsc::channel::<ReplToDaemon>(10);
    let (daemon_to_repl_tx, daemon_to_repl_rx) = mpsc::channel::<DaemonToRepl>(10);

    let registry = Arc::new(Mutex::new(Registry::new()));
    let storage = Arc::new(Mutex::new(Storage::default()));
    let pwd = Arc::new(Mutex::new(std::env::current_dir().unwrap_or_default()));

    // Start the REPL on a separate thread
    thread::spawn(move || {
        let mut repl = NadirRepl::new(registry.clone(), storage.clone(), pwd.clone());
        // TODO: make the daemon only start when necessary on command
        repl.connect_plot_daemon(repl_to_daemon_tx, daemon_to_repl_rx);
        if let Err(e) = repl.run() {
            eprintln!("{:?}", e)
        };
    });

    if let Err(e) = daemon(
        WindowManager::title,
        WindowManager::update,
        WindowManager::view,
    )
    .subscription(WindowManager::subscription)
    .run_with(|| WindowManager::new(daemon_to_repl_tx))
    {
        eprintln!("{:?}", e)
    };
}
