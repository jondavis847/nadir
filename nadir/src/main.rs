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
    Animate {
        /// Sim name in 'results'
        #[arg(short = 'r', long)]
        result: Option<String>,
    },
}

fn main() {
    let cli = Cli::parse();

    // Handle subcommands
    match cli.command {
        Commands::Animate { result } => {
            let pwd = std::env::current_dir().unwrap();
            let result_path = if let Some(result) = result {
                let results = pwd.join("results");
                if !results.is_dir() {
                    panic!("No 'results' folder found in pwd.");
                } else {
                    results.join(result)
                }
            } else {
                std::env::current_dir().unwrap()
            };
            animation::main(Some(result_path)).unwrap();
        }
    }
}
