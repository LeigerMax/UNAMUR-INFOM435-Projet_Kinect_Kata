using System.Windows;

namespace kinectKata
{
    /// <summary>
    /// Interaction logic for MessageWindow.xaml
    /// </summary>
    public partial class MessageWindow : Window
    {
        public MessageWindow()
        {
            InitializeComponent();
        }


        private void Leave_Click(object sender, RoutedEventArgs e)
        {
            Application.Current.Shutdown();
        }

 
        private void Restart_Click(object sender, RoutedEventArgs e)
        {
            this.Close(); 
        }
    }
}
