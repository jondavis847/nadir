use nadir_plots::PlotManager;
fn main() -> iced::Result {
    iced::daemon(PlotManager::title, PlotManager::update, PlotManager::view)
        .subscription(PlotManager::subscription)
        .run_with(PlotManager::new)
}
