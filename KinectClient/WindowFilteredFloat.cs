using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace KinectClient
{
    class WindowFilteredFloat
    {
        
        private List<float> _values;

        private int _w;
        public int W
        {
            get { return _w; }
            set { _w = value; }
        }

        private float _value;
        public float Value
        {
            get
            {
                if (_values.Count == 0)
                {
                    return 0;
                }
                else
                {
                    return _value;
                }
            }
            set { _update(value); }
        }

        public WindowFilteredFloat(int w)
        {
            _value = 0;
            _w = w;
            _values = new List<float>();
        }

        private void _update(float f)
        {
            if (_values.Count >= W)
            {
                _values.RemoveAt(0);
            }

            float m = 0;
            foreach (float i in _values)
            {
                m += i;
            }
            m += f;
            m = m / (_values.Count + 1);
            _values.Add(m);
            _value = m;
        }
    }
}
