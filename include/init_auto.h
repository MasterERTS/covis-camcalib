#ifndef INIT_AUTO_H
#define INIT_AUTO_H

#include <opencv2/core/core.hpp>
#include <algorithm>
#include <find_dots.h>


using namespace std;

double angle(const double &x0, const double &y0, const double &norm0, const double &x1, const double &y1)
{
    const double normProd = norm0*sqrt(x1*x1 + y1*y1);

    if(normProd == 0)
        return 2*M_PI;

    return atan2((x1*y0-x0*y1)/normProd, (x1*x0+y1*y0)/normProd);
}


unsigned int GetNextPoint(cv::Point _X0, const unsigned int &_idx, const std::vector<cv::Point> &_X, std::vector<unsigned int> &_spiral, double &_alpha, const bool &_first = false)
{

    const cv::Point X1 = _X[_idx];

    vector<double> dist;
    double v;
    vector<unsigned int> ngb;
    unsigned int idx_max;

    const unsigned int N = _first?5:5;

    for(unsigned int i=0;i<_X.size();++i)
    {
        if(std::find(_spiral.begin(), _spiral.end(), i) == _spiral.end())
        {
            v = (X1.x - _X[i].x)*(X1.x - _X[i].x) + (X1.y - _X[i].y)*(X1.y - _X[i].y);
            //cout << "   dist to " << i << ": " << v << endl;
            if(dist.size() < N)
            {
                if(!dist.size())
                    idx_max = 0;
                else if(v > dist[idx_max])
                    idx_max = dist.size();
                dist.push_back(v);
                ngb.push_back(i);
                //   //cout << " -> added" << endl;
            }
            else if(v < dist[idx_max])
            {
                // smaller than one already found
                //    //cout << " -> replace " << ngb[idx_max] << endl;
                dist[idx_max] = v;
                ngb[idx_max] = i;
                for(unsigned j=0;j<N;++j)
                {
                    if(dist[j] > dist[idx_max])
                        idx_max = j;
                }
            }
            /* else
                //cout << " -> ignored" << endl;*/
        }
    }

    /*  //cout << "  candidates: ";
    for(unsigned int i=0;i<ngb.size();++i)
        //cout << ngb[i] << ", ";
    //cout << endl;*/

    // ngb = idx of N nearest neighboors

    double alpha;
    const double x = _X0.x-X1.x;
    const double y = _X0.y-X1.y;
    const double nor = sqrt(x*x+y*y);
    unsigned int ind1;
    _alpha = 2*M_PI;
    bool ok;
    for(unsigned int i=0;i<ngb.size();++i)
    {
        if(_first)
        {
            alpha = -angle(x, y, nor, -_X[ngb[i]].x+X1.x, -_X[ngb[i]].y+X1.y);
            ok = alpha > 0;
        }
        else
        {
            alpha = abs(angle(x, y, nor, -_X[ngb[i]].x+X1.x, -_X[ngb[i]].y+X1.y));
            ok = true;
        }



        if(ok && (alpha < _alpha || abs(alpha-_alpha) < 0.2))
        {
            // check if almost the same
            if(abs(alpha-_alpha) < 0.2)
            {
                if(EuclideanDist(X1, _X[ind1]) > EuclideanDist(X1, _X[ngb[i]]))
                {
                    _alpha = alpha;
                    ind1 = ngb[i];
                }
            }
            else
            {
                _alpha = alpha;
                ind1 = ngb[i];
            }
        }
        //cout << " angle to " << ngb[i] << ": " << alpha << endl;

    }


    //cout << " winner: " << ind1 << endl;
    _spiral.push_back(ind1);
    return ind1;
}


// build the sequence of points
void InitAuto(std::vector<cv::Point> &_X)
{
    // if previous ordering is given simply take the nearest
   /* if(_Xprev.size() == -1)
    {
        vector<cv::Point> Xold = _X;
        unsigned int idx_min;
        double d_min, d;

        for(unsigned int i=0;i<36;++i)
        {
            d_min = 1000;
            // find nearest point
            for(unsigned int j=0;j<36;++j)
            {
                d = (_Xprev[j].x - Xold[i].x)*(_Xprev[j].x - Xold[i].x) + (_Xprev[j].y - Xold[i].y)*(_Xprev[j].y - Xold[i].y);
                if(d < d_min)
                {
                    d_min = d;
                    idx_min = j;
                }
            }
            _X[idx_min] = Xold[i];
        }
    }
    else*/
    {
        // first point: farest from the center of gravity
        double xCog=0, yCog=0;
        unsigned int i;

        for(i=0;i<_X.size();++i)
        {
            xCog += _X[i].x;
            yCog += _X[i].y;
        }

        xCog /= _X.size();
        yCog /= _X.size();
        cv::Point cog;cog.x = int(xCog);cog.y = int(yCog);

        double dMax = (xCog - _X[0].x)*(xCog - _X[0].x) + (yCog - _X[0].y)*(yCog - _X[0].y), d;
        unsigned int ind0 = 0;
        for(i=1;i<_X.size();++i)
        {
            d = (xCog - _X[i].x)*(xCog - _X[i].x) + (yCog - _X[i].y)*(yCog - _X[i].y);
            if(d > dMax)
            {
                d = dMax;
                ind0 = i;
            }
        }

        std::vector<unsigned int> spiral;
        spiral.push_back(ind0);
        // find other points from initiated sequence, counter-clockwise
        unsigned int ind1, ind2;
        bool check_corner = true;
        double alpha;
        while(spiral.size() < _X.size())
        {
            if(spiral.size() == 1)
            {
                //cout << "restarting from " << ind0 << endl;
                ind1 = GetNextPoint(cog, ind0, _X, spiral, alpha, true);
            }
            else
            {
                //cout << "starting from (" << ind0 << ", " << ind1 << ")" << endl;
                ind2 = GetNextPoint(_X[ind0], ind1, _X, spiral, alpha);
                ind0 = ind1;
                ind1 = ind2;
                // check corner, we may have begun on a side
                if(check_corner && alpha > M_PI/4)
                {
                    check_corner = false;
                    // reinit
                    spiral.clear();
                    spiral.push_back(ind0);
                }

            }

        }

        // build indices from spiral
        vector<unsigned int> idx(36);
        for(unsigned int i=0;i<36;++i)
        {
            //cout << spiral[i] << ", ";
            idx[i] = 36;
        }
        //cout << endl;
        int x=0, y=0, dx=1, dy=0;
        int xn=0,yn=0;
        bool turn;
        for(unsigned int i=0;i<36;++i)
        {
            turn = true;
            idx[x + 6*y] = spiral[i];
            //cout << "adding " << spiral[i] << " at (" << x << ", " << y << ")" << endl;
            xn = x+dx;
            yn = y+dy;
            // check if we stay on the grid
            if(xn>=0 && xn<6 && yn>=0 && yn<6)
            {
                if(idx[xn+6*yn] == 36)  // point was not written yet
                {
                    turn = false;
                }
            }
            if(turn)
            {
                //cout << "  turn" << endl;
                if(dy == 0)
                {
                    dy = dx;
                    dx = 0;
                }
                else
                {
                    dx = -dy;
                    dy = 0;
                }
                x = x+dx;
                y = y+dy;
            }
            else
            {
                x = xn;
                y = yn;
            }
        }

        //idx = spiral;

        // reorder
        vector<cv::Point> Xold = _X;
        for(unsigned int i=0;i<36;++i)
        {
            ////cout << idx[i] << ", ";
            _X[i] = Xold[idx[i]];
        }
        //  //cout << endl;
    }
}







#endif // INIT_AUTO_H
