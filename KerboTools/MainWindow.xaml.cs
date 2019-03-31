using KerboTools.ViewModels;
using KRPC.Client;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace KerboTools
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
        }

        private MissionViewModel model;
        private Connection conn;

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            this.conn = new Connection();
            this.model = new MissionViewModel(new ANewDawn.Mission.Mission(conn));
            this.DataContext = this.model;
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            this.DataContext = null;
            this.model.Dispose();
            this.model = null;
            this.conn.Dispose();
        }
    }
}
