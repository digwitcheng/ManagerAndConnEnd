using Agv.PathPlanning;
using AGV_V1._0.Agv;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AGV_V1._0.Algorithm
{
    class Astar
    {
       
        public const int SWERVE_COST = 3;
        public static int Search(Close [,]close, Open open,Node[,] graph,int beginX,int beginY,Direction beginDir)
        {    // A*算法遍历
            //int times = 0; 
            int i, curX, curY, surX, surY;
            float surG;
            Close curPoint = new Close();
            
            AstarUtil.push(open, close, beginX, beginY, 0);

            while (open.length > 0)
            {    //times++;
                curPoint = AstarUtil.shift(open);
                curX = curPoint.node.x;
                curY = curPoint.node.y;

                if (curPoint.from == null)
                {
                    curPoint.node.direction = beginDir;

                }
                else
                {
                    curPoint.node.direction = AstarUtil.getDirection(curPoint.from, curPoint);//0525
                }
                for (i = 0; i < 4; i++)
                {
                    if ((curPoint.node.adjoinNodeCount & (1 << i)) == 0)
                    {
                        continue;
                    }
                    surX = curX + (int)AstarUtil.dir[i].X;
                    surY = curY + (int)AstarUtil.dir[i].Y;
                    //if (surX < 0 || surY < 0)
                    //{
                    //    Console.WriteLine("走出场地外");
                    //    continue;
                    //} 
                    if (!close[surX, surY].vis)
                    {
                        close[surX, surY].vis = true;
                        close[surX, surY].from = curPoint;
                        Direction tempDir = new Direction();
                        int tempPassDifficulty = 2;
                        switch (i)
                        {
                            case 0:
                                tempDir = Direction.Right;
                                tempPassDifficulty = graph[curX, curY].rightDifficulty;
                                break;
                            case 1:
                                tempDir = Direction.Down;
                                tempPassDifficulty = graph[curX, curY].downDifficulty;
                                break;
                            case 2:
                                tempDir = Direction.Left;
                                tempPassDifficulty = graph[curX, curY].leftDifficulty;
                                break;
                            case 3:
                                tempDir = Direction.Up;
                                tempPassDifficulty = graph[curX, curY].upDifficulty;
                                break;
                        }
                        int directionCost = (tempDir == curPoint.node.direction) ? 0 : 1;
                        //  curPoint.node.stopTime = 1+directionCost * 2; 
                        int tempTraConges = graph[curX, curY].traCongesIntensity;


                        //curPoint.searchDir = close[surX, surY].searchDir;
                        surG = curPoint.G + (float)(Math.Abs(curX - surX) + Math.Abs(curY - surY)) + SWERVE_COST * (directionCost + tempTraConges) + tempPassDifficulty;
                        AstarUtil.push(open, close, surX, surY, surG);
                    }
                }
                if (curPoint.H == 0)
                {
                    return AstarUtil.Sequential;
                }
            }
            //System.Console.Write("times: %d\n", times);
            return AstarUtil.NoSolution; //无结果
        }


    }
}
