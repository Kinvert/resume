/*
	This was probably my third C++ project or so. Back in 2015.
	The code is a mess but it was a cool little project.
	A lot of debugging stuff was left in here since I basically only made this for a demo for Maker Faire Detroit 2015.

	The robot ran off C and it received the commands from the Raspberry Pi this ran on.
*/
#include <iostream>
//#include <iomanip> //old
#include <queue>
//#include <string> //old
#include <math.h> //old
//#include <ctime> //old

//#include <stdlib.h> //recently uncommented hoping to help with serial usleep
#include <cstdlib> //old
#include <stdio.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

#include <termios.h>
#include <sys/fcntl.h>
#include <unistd.h>

struct termios topt;
int ttyFid;

using namespace cv;
using namespace std;

bool light = false; //This was for 2 lighting configurations

int val1 = 0, val2 = 0, val3 = 0, val4 = 0, val5 = 0, val6 = 0; //for Trackbards
int asdf; //For map values
int c; //For waitkey values
int elem; //For erode/dilate
Mat out;
Mat out2;
Mat outb;
/*
    c = waitKey(0);
    if( (char)c == 27 )
    break;
*/

float binx = 39.0; //maze size in x direction in inches
float biny = 47.0; //maze size in y direction in inches

const int n=156; // horizontal size of the map in pixels
const int m=188; // vertical size size of the map in pixels
static int mapv[n][m]; //map matrix
static int closed_nodes_map[n][m]; // map of closed (tried-out) nodes
static int open_nodes_map[n][m]; // map of open (not-yet-tried) nodes
static int dir_map[n][m]; // map of directions
const int dir=8; // number of possible directions to go at any position
static int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1}; //dx directions
static int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1}; //dy directions

class node
{
    // current position
    int xPos;
    int yPos;
    // total distance already travelled to reach the node
    int level;
    // priority=level+remaining distance estimate
    int priority;  // smaller: higher priority

    public:
        node(int xp, int yp, int d, int p)
            {xPos=xp; yPos=yp; level=d; priority=p;}

        int getxPos() const {return xPos;}
        int getyPos() const {return yPos;}
        int getLevel() const {return level;}
        int getPriority() const {return priority;}

        void updatePriority(const int & xDest, const int & yDest)
        {
             priority=level+estimate(xDest, yDest)*10; //A*
        }

        // give better priority to going strait instead of diagonally
        void nextLevel(const int & i) // i: direction
        {
             level+=(dir==8?(i%2==0?10:14):10);
        }

        // Estimation function for the remaining distance to the goal.
        const int & estimate(const int & xDest, const int & yDest) const
        {
            static int xd, yd, d;
            xd=xDest-xPos;
            yd=yDest-yPos;

            // Euclidian Distance
            d=static_cast<int>(sqrt(xd*xd+yd*yd));

            // Manhattan distance
            //d=abs(xd)+abs(yd);

            // Chebyshev distance
            //d=max(abs(xd), abs(yd));

            return(d);
        }
};

// Determine priority (in the priority queue)
bool operator<(const node & a, const node & b)
{
  return a.getPriority() > b.getPriority();
}

// A-star algorithm.
// The route returned is a string of direction digits.
string pathFind( const int & xStart, const int & yStart,
                 const int & xFinish, const int & yFinish )
{

    //Mat path = cvCreateImage( Size(n,m), 8, 1 );
    //Mat::zeros( out.rows, out.cols, out.type() );
    Mat pathimg = Mat::zeros( m, n, CV_8UC1 );

    static priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
    static int pqi; // pq index
    static node* n0;
    static node* m0;
    static int i, j, x, y, xdx, ydy;
    static char c;
    pqi=0;

    // reset the node maps
    for(y=0;y<m;y++)
    {
        for(x=0;x<n;x++)
        {
            closed_nodes_map[x][y]=0;
            open_nodes_map[x][y]=0;
        }
    }

    // create the start node and push into list of open nodes
    n0=new node(xStart, yStart, 0, 0);
    n0->updatePriority(xFinish, yFinish);
    pq[pqi].push(*n0);
    open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes map

    // A* search
    while(!pq[pqi].empty())
    {
        // get the current node w/ the highest priority
        // from the list of open nodes
        n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
                     pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

        x=n0->getxPos(); y=n0->getyPos();

        pq[pqi].pop(); // remove the node from the open list
        open_nodes_map[x][y]=0;
        // mark it on the closed nodes map
        closed_nodes_map[x][y]=1;
        pathimg.at<uchar>(y,x) = 255;
        //imshow( "path", path );
        //waitKey(1);
        //usleep( 5 * 1000 );

        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if(x==xFinish && y==yFinish)
        {
            // generate the path from finish to start
            // by following the directions

            //imshow( "pathimg", pathimg );
            //cout << "AT THE FINISH" << endl;
            //waitKey(0);

            string path="";
            while(!(x==xStart && y==yStart))
            {
                j=dir_map[x][y];
                c='0'+(j+dir/2)%dir; //be careful I think c is used for waitkey
                path=c+path;
                x+=dx[j];
                y+=dy[j];
            }
            // garbage collection
            delete n0;
            // empty the leftover nodes
            while(!pq[pqi].empty()) pq[pqi].pop();
            cout << "RETURN PATH" << endl;
            return path;
        }

        // generate moves (child nodes) in all possible directions
        for(i=0;i<dir;i++)
        {
            xdx=x+dx[i]; ydy=y+dy[i];

            if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || mapv[xdx][ydy]==1
                || closed_nodes_map[xdx][ydy]==1))
            {
                // generate a child node
                m0=new node( xdx, ydy, n0->getLevel(),
                             n0->getPriority());
                m0->nextLevel(i);
                m0->updatePriority(xFinish, yFinish);

                // if it is not in the open list then add into that
                if(open_nodes_map[xdx][ydy]==0)
                {
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    pq[pqi].push(*m0);
                    // mark its parent node direction
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                }
                else if(open_nodes_map[xdx][ydy]>m0->getPriority())
                {
                    // update the priority info
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    // update the parent direction info
                    dir_map[xdx][ydy]=(i+dir/2)%dir;

                    // replace the node
                    // by emptying one pq to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(pq[pqi].top().getxPos()==xdx &&
                           pq[pqi].top().getyPos()==ydy))
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pq[pqi].pop(); // remove the wanted node

                    // empty the larger size pq to the smaller one
                    if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    while(!pq[pqi].empty())
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pqi=1-pqi;
                    pq[pqi].push(*m0); // add the better node instead
                }
                else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }
    return ""; // no route found
}

//-------------------------------------------------------------------------------------------------------------------------

void fake(int, void*) //refresh trackbar values
{
    val1 = val1;
    val2 = val2;
    val3 = val3;
    val4 = val4;
    val5 = val5;
    val6 = val6;
}

int main()
{

    int bhl1, bhh1, bhl2, bhh2, bsl, bsh, bvl, bvh;
    int rhl, rhh, rsl, rsh, rvl, rvh;
    int ed;
    elem = MORPH_RECT; //MORPH_CROSS   MORPH_ELLIPSE
    Mat element;

    srand(time(NULL));

    namedWindow( "win1", WINDOW_AUTOSIZE );
    createTrackbar("val1", "win1", &val1, 180, fake);
    createTrackbar("val2", "win1", &val2, 180, fake);
    createTrackbar("val3", "win1", &val3, 255, fake);
    createTrackbar("val4", "win1", &val4, 255, fake);
    createTrackbar("val5", "win1", &val5, 255, fake);
    createTrackbar("val6", "win1", &val6, 255, fake);
    Mat over = imread( "aaa4.jpg", CV_LOAD_IMAGE_COLOR ); //aaa aa2 big
    /*
    Mat over;
    waitKey(0);
    VideoCapture cap(0);
    if( !cap.isOpened() )
    {
        cout << "CAP NOT OPEN" << endl;
        return -1;
    }
    while(1)
    {
        for( int i=0; i<10; i++ ) cap >> over;
        imshow( "over", over );
        c = waitKey(0);
        if( (char)c == 27 ) break;
    }
    cap.release();
    imwrite( "aaa4.jpg", over );
    */

    cvtColor(over, over, CV_BGR2HSV );

    out = cvCreateImage( over.size(), 8, 1 );
    //namedWindow( "out", 0 );
    out2 = cvCreateImage( over.size(), 8, 1 );
    //namedWindow( "out2", 0 );
    outb = cvCreateImage( over.size(), 8, 1 );
    //namedWindow( "outb", 0 );
    Mat pass = cvCreateImage( Size(n,m), 8, 1 ); //over.size()
    namedWindow( "pass", 0 );


    int ohl1, ohh1, ohl2, ohh2, osl, osh, ovl, ovh;

    //Mat channel[3];
    //split( over, channel );
    //channel[1] = Mat::zeros( img.rows, img.cols, CV_8UC1 );
    //channel[2] = Mat::zeros( img.rows, img.cols, CV_8UC1 );
    //namedWindow( "h", 0 );
    //namedWindow( "s", 0 );
    //namedWindow( "v", 0 );
    //imshow( "h", channel[0] );
    //imshow( "s", channel[1] );
    //imshow( "v", channel[2] );
    if( !light )
    {
        ohl1 = 0; //0          0
        ohh1 = 10; //9          5
        osl  = 40; //120     167
        osh  = 255; //255      255
        ovl  = 0; //47        50
        ovh  = 255; //241      169
        ohl2 = 150; //170      174
        ohh2 = 180; //180      180
    }
    if( light )
    {
        ohl1 = val1;
        ohh1 = val2;
        osl  = val3;
        osh  = val4;
        ovl  = val5;
        ovh  = val6;
        ohl2 = 150;
        ohh2 = 180;
    }
//while(1)
//{
    //Scalar olb1(ohl1,osl,ovl,0); //lower bound for overview

    Scalar olb1(ohl1,osl,ovl,0);
    Scalar oub1(ohh1,osh,ovh,0);
    Scalar olb2(ohl2,osl,ovl,0);
    Scalar oub2(ohh2,osh,ovh,0);

    //Scalar olb1(val1,val3,val5,0);
    //Scalar oub1(val2,val4,val6,0);
    //Scalar olb2(ohl2,val3,val5,0);
    //Scalar oub2(ohh2,val4,val6,0);

    inRange( over, olb1, oub1, out ); //find blanket in overview
    inRange( over, olb2, oub2, outb );
    out = out + outb;                 //blanket with ribbon gaps and noise
    //imshow( "out", out );
    //c = waitKey(0);
    //if( (char)c == 27 )
    //break;
//}
    //imshow( "out", out );
    //waitKey(0);
    elem = MORPH_ELLIPSE;
    ed = 2; //remove some noise
    element = getStructuringElement( elem, Size(ed*2+1,ed*2+1), Point(ed,ed) );
    erode( out, out, element);
    //imshow( "out", out );
    //waitKey(0);
    ed = 2; //fill in ribbon gaps a bit
    element = getStructuringElement( elem, Size(ed*2+1,ed*2+1), Point(ed,ed) );
    dilate( out, out, element);
    //imshow( "out", out );
    //waitKey(0);

    //ed = 0; // shrink back down a bit
    //element = getStructuringElement( elem, Size(ed*2+1,ed*2+1), Point(ed,ed) );
    //erode( out, out, element);
    ed = 1;
    element = getStructuringElement( elem, Size(ed*2+1,ed*2+1), Point(ed,ed) );
    erode( out, out2, element); // out > pass
    elem = MORPH_RECT;

    //imshow( "out", out );

//=ifaeo

    vector<vector<Point> > cont;
    vector<Vec4i> hier;
    outb = Mat::zeros( out.rows, out.cols, out.type() );
    findContours( out, cont, hier, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );
    double bignew = contourArea( cont[0], false );
    double bigold = bignew;
    int bigcont = 0;
    for( int i = 0; i < cont.size(); i++ )
    {
        bignew = contourArea( cont[i], false );
        if( bignew > bigold )
        {
            bigcont = i;
            bigold = bignew;
        }
    }
    drawContours( outb, cont, bigcont, 255, 1, 8, hier, 0, Point() );
    vector<vector<Point> >hull(1);
    convexHull( Mat(cont[bigcont]), hull[0], false );
    drawContours( outb, hull, 0, 255, 2, 8, vector<Vec4i>(), 0, Point() );
    imshow( "outb", outb );
    imshow( "out2", out2 );
    waitKey(0);

//=============================================================================blanky

    vector<Point> approx;
    //approxPolyDP( Mat(cont[bigcont]), approx, 25, true );//tk 50 needs to change possibly
    approxPolyDP( Mat(hull[0]), approx, 25, true );
    if( approx.size() > 4 )
    {
        cout << " More than four corners!!!" << endl;
        return -1;
    }
    if( approx.size() < 4 )
    {
        cout << " Less than four corners!!!" << endl;
        return -1;
    }

    int ind[4], tempint;
    for( size_t i = 0; i < approx.size(); i++ )  //create index
    {
        ind[i] = i;
    }
    for( size_t i = 0; i < approx.size(); i++ ) //make index in order of highest y to lowest y
    {
        for( size_t j = 0; j < approx.size(); j++ )
        {
            if( approx[ind[i]].y > approx[ind[j]].y )
            {
                tempint = ind[j];
                ind[j] = ind[i];
                ind[i] = tempint;
            }
        }
    }
    /*
    for( size_t i = 0; i < approx.size()-1; i++ ) //sort x
    {
        for( size_t j = i+1; j < approx.size()-2; j++ )
        {
            if( approx[ind[i]].x > approx[ind[j]].x )
            {
                tempint = ind[j];
                ind[j] = ind[i];
                ind[i] = tempint;
            }
        }
    }
    */

    if( approx[ind[0]].x > approx[ind[1]].x )
    {
        tempint = ind[1];
        ind[1] = ind[0];
        ind[0] = tempint;
    }
    if( approx[ind[2]].x > approx[ind[3]].x )
    {
        tempint = ind[3];
        ind[3] = ind[2];
        ind[2] = tempint;
    }

    /*
    for( size_t i = 0; i < approx.size(); i++ ) //draw circles to visually check the sort
    {
        circle( out, approx[ind[i]], 5, 128, 5, 8, 0 );
        imshow( "out", out );
        waitKey(0);
    }
    */

    for( size_t i = 0; i < approx.size()-1; i++ )
    {
        line(outb, approx[i], approx[i+1], 255, 2, 8, 0 );
    }
    line(outb, approx[0], approx[approx.size()-1], 255, 2, 8, 0 );
    //drawContours( outb, cont, bigcont, 255, 2, 8, hier, 0, Point() );
    imshow( "outb", outb );

    Point2f inq[4];
    Point2f outq[4];
    Mat lam( 2, 4, CV_32FC1 );
    lam = Mat::zeros( over.rows, over.cols, over.type() );

    inq[0]  = Point2f( approx[ind[0]].x, approx[ind[0]].y );
    inq[1]  = Point2f( approx[ind[1]].x, approx[ind[1]].y );
    inq[2]  = Point2f( approx[ind[2]].x, approx[ind[2]].y );
    inq[3]  = Point2f( approx[ind[3]].x, approx[ind[3]].y );

                     //640  480
    outq[0] = Point2f(   0,   m );
    outq[1] = Point2f(   n,   m );
    outq[2] = Point2f(   0,   0 );
    outq[3] = Point2f(   n,   0 );
/*
    imshow( "over", over );
    for( size_t i=0 ; i<4 ; i++ )
    {
    circle( out, inq[i], 20, 128, 5, 8, 0 );
    circle( out, outq[i], 20, 128, 20, 8, 0 );
    imshow( "out", out );
    waitKey(0);
    }
*/
    lam = getPerspectiveTransform( inq, outq );

    //resize( pass, pass, Size(n,m), 0, 0, INTER_LINEAR );

    warpPerspective( out2, pass, lam, over.size() );
    Rect myROI(0,0,n,m);
    pass = pass(myROI);
    bitwise_not( pass, pass );
    Mat map = Mat::zeros( pass.rows, pass.cols, CV_8UC1 );

    vector<Vec3f> circles;
    HoughCircles( pass, circles, CV_HOUGH_GRADIENT, 1, 20, 255, 12, n/40, n/7 );
    imshow( "pass", pass );
    waitKey(0);
    for( size_t i=0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]) );
        int radius = cvRound(circles[i][2]);
        circle( pass, center, n/6, 0, -1, 8, 0 );
        circle( map, center, 3, 255, -1, 8, 0 );
        cout << "CENTER = " << center << "    Radius = " << radius << endl;
    }
    if( circles.size() > 2 || circles.size() < 2 )
    {
        cout << "Found " << circles.size() << " circles!!!" << endl;
        imshow( "pass", pass );
        imshow( "map", map );
        waitKey(0);
        return -1;
    }

    line(pass, Point(0,0), Point(n,0), 0, n/20, 8, 0 );
    line(pass, Point(n,0), Point(n,m), 0, n/20, 8, 0 );
    line(pass, Point(n,m), Point(0,m), 0, n/20, 8, 0 );
    line(pass, Point(0,m), Point(0,0), 0, n/20, 8, 0 );

    /*
    for ( int xx = 0; xx < n; xx++ ) //n
    {
        for (int yy = 0; yy < m; yy++) //m
        {
            if( yy==0 || yy==m ) pass.at<uchar>(yy,xx) = 255;
            if( xx==0 || xx==n ) pass.at<uchar>(yy,xx) = 255;
        }
    }
    */

    imshow( "pass", pass );
    waitKey(0);
    ed = cvRound(n/20); //fill in ribbon gaps a bit
    elem = MORPH_RECT;
    element = getStructuringElement( elem, Size(ed*2+1,ed*2+1), Point(ed,ed) );
    dilate( pass, pass, element);
    imshow( "pass", pass );
    waitKey(0);


//=ojioj

    //resize( pass, pass, Size(n,m), 0, 0, INTER_LINEAR );
    //resize( out2, out2, Size(n,m), 0, 0, INTER_LINEAR );
    int xA,yA,xB,yB;
    xB = cvRound(circles[1][0]);//n-10
    yB = cvRound(circles[1][1]);//10
    xA = cvRound(circles[0][0]);//10
    yA = cvRound(circles[0][1]);//m-10
    if( circles[1][1] > circles[0][1] )
    {
        xB = cvRound(circles[0][0]);//n-10
        yB = cvRound(circles[0][1]);//10
        xA = cvRound(circles[1][0]);//10
        yA = cvRound(circles[1][1]);//m-10
    }

    for ( int xx = 0; xx < n; xx++ ) //n
    {
        for (int yy = 0; yy < m; yy++) //m
        {
            asdf = pass.at<uchar>(yy,xx);
            mapv[xx][yy] = asdf/255;
            if( xx<xA+5 && xx>xA-5 && yy<yA+5 && yy>yA-5 ) mapv[xx][yy] = 0;
            if( yy==0 || yy==m ) mapv[xx][yy] = 1;
            if( xx==0 || xx==n ) mapv[xx][yy] = 1;
        }
    }
    // get the route
    cout << "Finding path...." << endl;
    string route=pathFind(xA, yA, xB, yB);
    if(route=="") cout<<"An empty route generated!"<<endl;

    int pathordx[1024];
    int pathordy[1024];
    vector<Point> pathord;

    // follow the route on the map and display it
    if(route.length()>0)
    {
        int j; char c;
        int ex=xA;
        int ey=yA;
        mapv[ex][ey]=2;
        for(int i=0 ; i<route.length() ; i++)
        {
            c =route.at(i);
            j=atoi(&c);
            if( j > 8 ) j = j / 10;
            ex=ex+dx[j];
            ey=ey+dy[j];
            mapv[ex][ey]=3;
            pathordx[i] = ex;
            pathordy[i] = ey;
            pathord.push_back(Point(ex,ey) );
        }
        mapv[ex][ey]=4;
//===============================================================================================
        // display the map with the route

        for( int yyy=1; yyy < m-1; yyy++ )
        {
            for( int xxx=1; xxx < n-1; xxx++ )
            {
                //cout << "(" << xxx << "," << yyy << ")" << "MAPV[x][y] = " << mapv[xxx][yyy] << endl;
                //waitKey(0);

                if(mapv[xxx][yyy]==3)
                {
                    //cvSet2D(img,y,x,Scalar(128,128,128,0));
                    //img.at<Vec3b>(y,x)[0] = img.at<Vec3b>(y,x)[1] = img.at<Vec3b>(y,x)[2] =  128;
                    map.at<uchar>(yyy,xxx) = 255;
                }
            }
        }

/*

        for(int y=0;y<m;y++)
        {
            for(int x=0;x<n;x++)
                if(mapv[x][y]==0)
                    cout<<".";
                else if(mapv[x][y]==1)
                    cout<<"O"; //obstacle
                else if(mapv[x][y]==2)
                    cout<<"S"; //start
                else if(mapv[x][y]==3)
                {
                    cout<<"R"; //route
                    //cvSet2D(img,y,x,Scalar(128,128,128,0));
                    //img.at<Vec3b>(y,x)[0] = img.at<Vec3b>(y,x)[1] = img.at<Vec3b>(y,x)[2] =  128;
                    //map.at<uchar>(y,x) = 255;
                }
                else if(mapv[x][y]==4)
                    cout<<"F"; //finish
            cout<<endl;
        }

*/

    vector<Point> corners;
    vector<Point> actord;
    int maxCorners = 10;
    double quallev = 0.01;
    double mindist = n/8.0;//10.0
    int blocksize = 3;
    bool usehar = false;
    double kk = 0.04;
    Mat good;
    goodFeaturesToTrack( map, corners, maxCorners, quallev, mindist, good, blocksize, usehar, kk );

    namedWindow( "map", 0 );

    /*for( size_t i = 0; i < corners.size(); i++ )
    {
        circle( map, corners[i], 5, 255, 2, 8, 0 );
        imshow( "map", map );
        waitKey(0);
    }*/

    for( size_t i = 0; i < pathord.size(); i++ )
    {
        for( size_t j = 0; j < corners.size(); j++ )
        {
            //if( corners[j] == pathord[i] ) actord.push_back( corners[j] );
            if( norm(pathord[i]-corners[j]) < 5 )
            {
                actord.push_back( corners[j] );
                i = i + mindist;//10
                //j = -1;

                //if( actord[ actord.size()-1 ] == actord[ actord.size() ] )
                //{
                //    actord.pop_back();
                //    cout << "POP BACK" << endl;
                //}
            }
        }
    }

    pass = pass + map;

    //for( size_t i = 0; i < corners.size(); i++ )
    for( size_t i = 0; i < actord.size(); i++ )
    {
        //circle( map, corners[i], 5, 255, 1, 8, 0 );
        circle( map, actord[i], 10, 255, 1, 8, 0 );
        imshow( "map", map );
        //cout << corners[i] << "  ,  " << pathord[i] << "ACTORD = " << actord[i] <<  endl;
        //waitKey(0);
    }
    namedWindow( "map", 0 );
    imshow( "map", map );
    imshow( "pass", pass );

    cout << "ABOUT TO TRY SERIAL" << endl;
    waitKey(0);


    ttyFid = open( "/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC); // ttyUSB0 ttyAMA0
    memset( &topt, 0, sizeof(topt) );
    if(ttyFid == -1)
    {
        printf( "Unable to open port keith\n" );
        return -1;
    }

    cfsetispeed( &topt, B9600);
    cfsetospeed( &topt, B9600);
    topt.c_cflag = (topt.c_cflag & ~CSIZE) | CS8;
    topt.c_iflag &= ~IGNBRK;
    topt.c_lflag = 0;
    topt.c_oflag = 0;
    topt.c_cc[VMIN] = 0;
    topt.c_cc[VTIME] = 5;
    topt.c_iflag &= ~(IXON | IXOFF | IXANY );
    topt.c_cflag |= (CLOCAL | CREAD);
    topt.c_cflag &= ~(PARENB | PARODD);
    topt.c_cflag |= 0; //parity
    topt.c_cflag &= ~CSTOPB;
    //topt.c_cflag &= ~CSIZE;
    //topt.c_cflag |= CS8;
    topt.c_cflag &= ~CRTSCTS;

    //if( tcgetattr( ttyFid, &topt ) != 0 );
    //{
        //printf("Couldn't get Term Attributes keith %d\n", errno);
        //return -1;
    //}

    tcgetattr( ttyFid, &topt );
    tcsetattr( ttyFid, TCSANOW, &topt );
    if( tcsetattr( ttyFid, TCSAFLUSH, &topt ) < 0 )
    {
        printf("Set Term Att Fail keith\n");
        return -1;
    }

    /*
    for( size_t i = 0; i < actord.size(); i++ )
    {
        //circle( map, corners[i], 5, 255, 1, 8, 0 );
        circle( map, actord[i], 10, 255, 1, 8, 0 );
        imshow( "map", map );
        //cout << corners[i] << "  ,  " << pathord[i] << "ACTORD = " << actord[i] <<  endl;
        //waitKey(0);
    }
    */


    float ppix = n/binx;
    float ppiy = m/biny;
    float dxi[actord.size()-1];
    float dyi[actord.size()-1];
    //float dist[actord.size()-1];
    //float ang[actord.size()-2];
    int dist[actord.size()-1];
    int ang[actord.size()-2];

    float angba;
    float angbc;
    float rslt;

    for( size_t i = 0; i < actord.size(); i++ )
    {
        cout << actord[i] << endl;
    }

    for( size_t i = 0; i < actord.size()-1; i++ )
    {
        dxi[i] = (actord[i+1].x - actord[i].x) / ppix;
        dyi[i] = (actord[i+1].y - actord[i].y) / ppiy;
        dist[i] = sqrt( dxi[i]*dxi[i] + dyi[i]*dyi[i] )*4;
        printf( "deltax = %f, deltay = %f, distance = %d \n", dxi[i], dyi[i], dist[i] );
    }


    for( size_t i = 1; i < actord.size()-1; i++ )
    {
        angba = atan2( dyi[i-1] , dxi[i-1] );
        angbc = atan2( dyi[i]   , dxi[i]   );
        rslt = angba - angbc;
        ang[i-1] = 2*(rslt * 180) / (3.1426);
        printf( "angba = %f, angbc = %f, rslt = %f, angle = %d\n", angba, angbc, rslt, ang[i-1] );
    }

    char ch;
    int floatchar;
    ch = ( actord.size() );
    write( ttyFid, &ch, sizeof(ch) );
    usleep( 100 * 1000 );

    angbc = atan2( dyi[0], dxi[0] );
    rslt = 0 - angbc;
    floatchar = 2*(rslt * 180) / (3.1426);

    if( floatchar > 0 )
    {
        ch = 3;
        write( ttyFid, &ch, sizeof(ch) );
        usleep( 100 * 1000 );
        ch = abs(floatchar);
        write( ttyFid, &ch, sizeof(ch) );
        usleep( 100 * 1000 );
    }
    if( floatchar < 0 )
    {
        ch = 2;
        write( ttyFid, &ch, sizeof(ch) );
        usleep( 100 * 1000 );
        ch = abs(floatchar);
        write( ttyFid, &ch, sizeof(ch) );
        usleep( 100 * 1000 );
    }

    ch = 1;
    write( ttyFid, &ch, sizeof(ch) );
    usleep( 100 * 1000 );
    ch = dist[0];
    write( ttyFid, &ch, sizeof(ch) );
    usleep( 100 * 1000 );

    for( size_t i = 1; i < actord.size()-1; i++ ) //-2
    {
        if( ang[i-1] > 0 )
        {
            ch = 3;
            write( ttyFid, &ch, sizeof(ch) );
            usleep( 100 * 1000 );
            ch = abs(ang[i-1]); //i
            write( ttyFid, &ch, sizeof(ch) );
            usleep( 100 * 1000 );
        }
        if( ang[i-1] < 0 )
        {
            ch = 2;
            write( ttyFid, &ch, sizeof(ch) );
            usleep( 100 * 1000 );
            ch = abs(ang[i-1]); //i
            write( ttyFid, &ch, sizeof(ch) );
            usleep( 100 * 1000 );
        }
        ch = 1;
        write( ttyFid, &ch, sizeof(ch) );
        usleep( 100 * 1000 );
        ch = dist[i];
        write( ttyFid, &ch, sizeof(ch) );
        usleep( 100 * 1000 );
    }

    getchar();
    element.release();
    out.release();
    out2.release();
    outb.release();
    map.release();
    over.release();
    destroyAllWindows();
    }
    getchar();
    close( ttyFid );
    return 0;
}

/* ifaeo
    rhl = 90; //90
    rhh = 105; //120
    rsl = 185; //0
    rsh = 255; //255
    rvl = 0; //0
    rvh = 255; //255

    Scalar rlb(rhl,rsl,rvl,0);
    Scalar rub(rhh,rsh,rvh,0);
    inRange( over, rlb, rub, outb ); //find ribbons

    ed = 2; //remove noise so only ribbons remain
    element = getStructuringElement( elem, Size(ed*2+1,ed*2+1), Point(ed,ed) );
    erode( outb, outb, element);
    ed = 7; //enlarge ribbons
    element = getStructuringElement( elem, Size(ed*2+1,ed*2+1), Point(ed,ed) );
    dilate( outb, outb, element);
    ed = 3; // shrink back down just a bit
    element = getStructuringElement( elem, Size(ed*2+1,ed*2+1), Point(ed,ed) );
    erode( outb, outb, element);
*
    //out = out + outb; //overlap ribbons and blanket
    //ed = 3;
    //element = getStructuringElement( elem, Size(ed*2+1,ed*2+1), Point(ed,ed) );
    //dilate( out, out, element); //fill in any gaps between ribbons and blanket

//==========================BAD PARAGRAPH========================BAD PARAGRAPH========
/*
    ed = 50; //poor attempt at removing any ribbon overlap
    element = getStructuringElement( elem, Size(ed*2+1,ed*2+1), Point(ed,ed) );
    erode( out, outb, element);
    dilate( outb, outb, element);
ifaeo */






/* ojioj

    Mat img = cvCreateImage( over.size(), 8, 3 );
    Mat img2;
    //Mat channel[3];
    namedWindow( "win2", 0);
    namedWindow( "win3", 0);
    cvtColor(img, img, CV_BGR2HSV );
    //split( img, channel );
    //channel[1] = Mat::zeros( img.rows, img.cols, CV_8UC1 );
    //channel[2] = Mat::zeros( img.rows, img.cols, CV_8UC1 );
    //namedWindow( "h", 0 );
    //namedWindow( "s", 0 );
    //namedWindow( "v", 0 );
    //imshow( "h", channel[0] );
    //imshow( "s", channel[1] );
    //imshow( "v", channel[2] );
//while(1)
//{

    rhl = 90; //90
    rhh = 120; //120
    rsl = 0; //0
    rsh = 255; //255
    rvl = 0; //0
    rvh = 255; //255

    out = out + outb;
    //MORPH_ELLIPSE RECT CROSS
    element = getStructuringElement( elem, Size(val1*2+1,val1*2+1), Point(val1,val1) );
    erode( out, out, element);
    element = getStructuringElement( elem, Size(val2*2+1,val2*2+1), Point(val2,val2) );
    dilate( out, out, element);
    erode( out, out, element); //blanket mask is ready

    //img.copyTo( img2, out);
    bitwise_and(img, img, img2, out); // src1, src2, dst, mask

    //Scalar rlb(rhl,rsl,rvl,0);
    //Scalar rub(rhh,rsh,rvh,0);
    //inRange( img2, rlb, rub, img2 );

    element = getStructuringElement( elem, Size(val3*2+1,val3*2+1), Point(val3,val3) );
    erode( img2, img2, element);
    element = getStructuringElement( elem, Size(val4*2+1,val4*2+1), Point(val4,val4) );
    dilate( img2, img2, element);
    element = getStructuringElement( elem, Size(val5*2+1,val5*2+1), Point(val5,val5) );
    erode( img2, img2, element);

    out = 255 - out;
    out2 = img2 + out;
    //c = waitKey(0);
    //if( (char)c == 27 ) break;
    //else if( (char)c == 'r' ) elem = MORPH_RECT;
    //else if( (char)c == 'c' ) elem = MORPH_CROSS;
    //else if( (char)c == 'e' ) elem = MORPH_ELLIPSE;
//}
    Mat map;
    //map = Mat( 60, 80, CV_8UC1, cvScalar(0) );
    namedWindow( "map", 0 );

ojioj */








/* //-------------------------------------------------------------------------------SERIAL
    ttyFid = open( "/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC); // ttyUSB0 ttyAMA0
    memset( &topt, 0, sizeof(topt) );
    if(ttyFid == -1)
    {
        printf( "Unable to open port keith\n" );
        return -1;
    }

    cfsetispeed( &topt, B9600);
    cfsetospeed( &topt, B9600);
    topt.c_cflag = (topt.c_cflag & ~CSIZE) | CS8;
    topt.c_iflag &= ~IGNBRK;
    topt.c_lflag = 0;
    topt.c_oflag = 0;
    topt.c_cc[VMIN] = 0;
    topt.c_cc[VTIME] = 5;
    topt.c_iflag &= ~(IXON | IXOFF | IXANY );
    topt.c_cflag |= (CLOCAL | CREAD);
    topt.c_cflag &= ~(PARENB | PARODD);
    topt.c_cflag |= 0; //parity
    topt.c_cflag &= ~CSTOPB;
    //topt.c_cflag &= ~CSIZE;
    //topt.c_cflag |= CS8;
    topt.c_cflag &= ~CRTSCTS;

    //if( tcgetattr( ttyFid, &topt ) != 0 );
    //{
        //printf("Couldn't get Term Attributes keith %d\n", errno);
        //return -1;
    //}

    tcgetattr( ttyFid, &topt );
    tcsetattr( ttyFid, TCSANOW, &topt );
    if( tcsetattr( ttyFid, TCSAFLUSH, &topt ) < 0 )
    {
        printf("Set Term Att Fail keith\n");
        return -1;
    }
    char ch;
    ch = 2; //1F 2R 3L
    write( ttyFid, &ch, sizeof(ch) );
    usleep(10000);
    ch = 45; //quarters of inch or 2 degrees, so 4 for 1 inch and 45 for 90 degrees
    write( ttyFid, &ch, sizeof(ch) );
    usleep(10000);
    ch = 1;
    write( ttyFid, &ch, sizeof(ch) );
    usleep(10000);
    ch = 8;
    write( ttyFid, &ch, sizeof(ch) );
*/ //-------------------------------------------------------SERIAL


/*
#include "opencv2/core/core_c.h"
//#include "opencv2/core/core.hpp"
//#include "opencv2/flann/miniflann.hpp"
#include "opencv2/imgproc/imgproc_c.h"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/photo/photo.hpp"
//#include "opencv2/video/video.hpp"
//#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/objdetect/objdetect.hpp"
//#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/ml/ml.hpp"
#include "opencv2/highgui/highgui_c.h"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/contrib/contrib.hpp"

#include "opencv2/legacy/legacy.hpp"
//#include "opencv2/legacy/compat.hpp"
*/

/*
vector<Vec2f> lines;
    HoughLines( map, lines, 10, CV_PI/90, 5, 0, 0 );
    imshow( "win3", out2 );
    cout << "lines" << lines.size() << endl;
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float theta = lines[i][1];
        float rho = lines[i][0];
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        Point pt1(cvRound(x0 + 1000*(-b)),cvRound(y0 + 1000*(-b)));
        Point pt2(cvRound(x0 - 1000*(-b)),cvRound(y0 - 1000*(-b)));
        line( out2, pt1, pt2, 255, 1, 8 );
    }
*/

/*
    Mat skel(out2.size(), CV_8UC1, Scalar(0));
    Mat temp(out2.size(), CV_8UC1 );
    Mat outinv(out2.size(), CV_8UC1 );
    bitwise_not( out2, outinv );
    element = getStructuringElement( MORPH_CROSS, Size(3,3) );
    bool done;
    do
    {
        morphologyEx( outinv, temp, MORPH_OPEN, element);
        bitwise_not( temp, temp);
        bitwise_and( outinv, temp, temp );
        bitwise_or(skel, temp, skel );
        erode( outinv, outinv, element );

        double max;
        minMaxLoc( outinv, 0, &max );
        done = (max == 0 );
    } while (!done);
    temp = skel + out2;
    namedWindow( "temp", 0 );
    namedWindow( "skel", 0 );
    imshow( "temp", temp );
    imshow( "skel", skel );
    imshow( "win3", out2 );
    waitKey(0);
*/

/* //CONTOURS------------------------------------------------CONTOURS
    CvMemStorage* stc = cvCreateMemStorage(0);
    CvMemStorage* std = cvCreateMemStorage(0);
    vector<vector<Point> > cont;
    vector<vector<Point> > dps;
    vector<Vec4i> hier;
    findContours( map, cont, hier, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );
    for( int i = 0; i < cont.size(); i++ )
    {
    drawContours( map, cont, i, 255, 2, 8, hier, 0, Point() );
    }
*/ //CONTOURS------------------------------------------------CONTOURS

