using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace KinectClient
{
    class UdpBroadcast
    {
        private string _address;
        private int _port;

        private IPEndPoint _remoteEndPoint;
        private UdpClient _udp;

        public UdpBroadcast(string address, int port)
        {
            _address = address;
            _port = port;

            
            _remoteEndPoint = new IPEndPoint(IPAddress.Parse(_address), _port);
            _udp = new UdpClient();

        }

        public void send(string line)
        {
            try
            {
                byte[] data = Encoding.UTF8.GetBytes(line);
                _udp.Send(data, data.Length, _remoteEndPoint);

            } 
            catch (Exception e)
            {
                System.Diagnostics.Debug.WriteLine("[UDP] " + e.Message);
            }
        }


    }
}
