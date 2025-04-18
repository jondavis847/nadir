use crossbeam::channel::{Receiver, Sender, unbounded};
use egui::{Context, Pos2, ViewportBuilder, ViewportId};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::thread;

#[derive(Default)]
pub struct PlotManager {
    plots: HashMap<u32, Plot>,
}

impl PlotManager {
    pub fn get_id(&self) -> u32 {
        (0..).find(|key| !self.plots.contains_key(key)).unwrap_or(0)
    }
}
pub struct Plot {}

// Define your plot commands
pub enum PlotCommand {
    NewWindow(u32),
    // Other plotting commands...
}

pub struct PlotApp {
    // Thread-safe storage for commands
    pending_window_commands: Arc<Mutex<Vec<PlotCommand>>>,
    // Keep the receiver
    plot_command_rx: Receiver<PlotCommand>,
}

impl PlotApp {
    pub fn new(plot_command_rx: Receiver<PlotCommand>) -> Self {
        Self {
            pending_window_commands: Arc::new(Mutex::new(Vec::new())),
            plot_command_rx,
        }
    }

    // Check for new commands from the channel
    fn process_channel_messages(&mut self) {
        while let Ok(command) = self.plot_command_rx.try_recv() {
            if let Ok(mut commands) = self.pending_window_commands.lock() {
                commands.push(command);
            }
        }
    }

    // Process any pending window creation commands
    fn process_pending_windows(&mut self, ctx: &Context) {
        let mut commands_to_process = Vec::new();

        // Extract commands from the mutex to minimize lock time
        if let Ok(mut commands) = self.pending_window_commands.lock() {
            std::mem::swap(&mut commands_to_process, &mut commands);
        }

        // Process each pending window command
        for command in commands_to_process {
            match command {
                PlotCommand::NewWindow(id) => {
                    // Create a unique ID for this viewport
                    let viewport_id = ViewportId::from_hash_of(id);

                    // Configure your viewport
                    let viewport_builder = ViewportBuilder::default()
                        .with_inner_size([800.0, 400.0])
                        .with_position(Pos2::new(100.0, 100.0)); // Or position dynamically

                    // Create the deferred viewport
                    ctx.show_viewport_deferred(
                        viewport_id,
                        viewport_builder,
                        move |ctx, _class| {
                            // This closure will be called to render the contents of your window
                            egui::CentralPanel::default().show(ctx, |ui| {
                                // Here's where you would do your actual plotting
                                ui.heading("Plot Window");
                                ui.label("Your plot content will go here");

                                // Add your plotting code...
                            });
                        },
                    );
                    dbg!("test");
                }
            }
        }
    }

    // Main update function that gets called each frame
    fn update(&mut self, ctx: &Context) {
        // First check for new commands
        self.process_channel_messages();

        // Then process any pending window commands
        self.process_pending_windows(ctx);

        // Your regular UI code
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("Main window");
            // Rest of your UI...
        });
    }
}

// Implement the eframe::App trait
impl eframe::App for PlotApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.update(ctx);
    }
}
