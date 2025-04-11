use clap::{Parser, Subcommand};

/// A program named `nadir` for various operations
#[derive(Parser, Debug)]
#[command(name = "NADIR")]
#[command(version = "0.1")]
#[command(about = "NASA Attitude Dynamics In Rust", long_about = None)]
struct Cli {
    /// Activate verbose mode
    #[arg(short, long, global = true)]
    verbose: bool,

    /// Subcommands for specific operations
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// Animates a NADIR result
    Animate,
    /// Plot a NADIR result
    Plot,
}

fn main() {
    let cli = Cli::parse();

    // Handle subcommands
    match cli.command {
        Commands::Animate => {
            let result_path = std::env::current_dir().unwrap();
            animation::main(Some(result_path)).unwrap();
        }
        Commands::Plot => {
            let result_path = std::env::current_dir().unwrap();
            plotting::main(Some(result_path));
        }
    }
}
