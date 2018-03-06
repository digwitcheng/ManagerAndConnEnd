using Agv.PathPlanning;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Algorithm
{
    class Bfs
    {         
        public static int Search(Close[,] close,  Node[,] graph, int beginX, int beginY,int Height,int Width)
        {
            int times = 0;
            int i, curX, curY, surX, surY;
            int f = 0, r = 1;
            Close p = new Close();
            Close[] q = new Close[Height * Width];
            int w = 0;
            for (int m = 0; m < Height; m++)
            {
                for (int n = 0; n < Width; n++)
                {
                    q[w] = new Close ();
                    w++;
                }
            }
            close[beginX, beginY].vis = true;

            while (r != f)
            {
                p = q[f];
                f = (f + 1) % AstarUtil.MaxLength;
                curX = p.node.x;
                curY = p.node.y;
                for (i = 0; i < 8; i++)
                {
                    if ((p.node.adjoinNodeCount & (1 << i)) == 0)
                    {
                        continue;
                    }
                    surX = curX + (int)AstarUtil.dir[i].X;
                    surY = curY + (int)AstarUtil.dir[i].Y;
                    if (surX < 0 || surY < 0)
                    {
                        Console.WriteLine("走出场地外");
                        continue;
                    }
                    if (!close[surX, surY].vis)
                    {
                        close[surX, surY].from = p;
                        close[surX, surY].vis = true;
                        close[surX, surY].G = p.G + 1;
                        q[r] = close[surX, surY];
                        r = (r + 1) % AstarUtil.MaxLength;
                    }
                }
                times++;
            }
            return times;
        }
    }
}
