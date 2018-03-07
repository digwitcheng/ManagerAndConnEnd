using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.Serialization;
using System.Text;
using System.Threading.Tasks;

namespace AGVSocket.Network
{
    class CellPoint
    {
        private uint x;
        private uint y;

        public uint Y
        {
            get { return y; }
            set { y = value; }
        }

        public uint X
        {
            get { return x; }
            set { x = value; }
        }

        public CellPoint(CellPoint poUInt32)
        {
            if (poUInt32 == null)
            {
                return;
            }
            this.x = poUInt32.x;
            this.y = poUInt32.y;

        }
        public CellPoint(uint x, uint y)
        {
            this.x = x;
            this.y = y;
        }
        public CellPoint(int x, int y)
        {
            this.x = (uint)x;
            this.y = (uint)y;
        }
    }
}
