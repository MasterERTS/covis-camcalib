#ifndef DISTORTION_CAMERA_H
#define DISTORTION_CAMERA_H

#include <generic_camera.h>
#include <visp/vpFeaturePoint3D.h>

namespace covis
{

// distortion camera class, from ICRA 2016 paper
// xi = (px, py, u0, v0, alpha, beta)
class DistortionCamera : public GenericCamera
{
public:

    vpFeaturePoint3D p_;
    vpMatrix dPdX_;

    DistortionCamera(const double &_px, const double &_py, const double &_u0, const double &_v0, const double &_alpha, const double &_beta)
    {
        dPdX_.resize(2,3);
        xi_.resize(6);
        xi_[0] = _px;
        xi_[1] = _py;
        xi_[2] = _u0;
        xi_[3] = _v0;
        xi_[4] = _alpha;
        xi_[5] = _beta;
    }


    // compute pixel coordinates of a 3D point
    // we assume the point is already in the camera frame
    void project(const vpPoint &_P, double &_u, double &_v)
    {
        const double rho = sqrt(xi_[5]*(_P.get_X()*_P.get_X()+_P.get_Y()*_P.get_Y())+_P.get_Z()*_P.get_Z());
        const double nu_inv = 1./(xi_[4]*rho + (1-xi_[4])*_P.get_Z());
        _u = xi_[0]*_P.get_X()*nu_inv + xi_[2];    // u = px.X/nu + u0
        _v = xi_[1]*_P.get_Y()*nu_inv + xi_[3];    // v = py.Y/nu + v0
    }

    // write the Jacobian corresponding to the intrinsic parameters
    // Jacobian should be 2x6
    void computeJacobianIntrinsic(const vpPoint &_P, vpMatrix &_J)
    {
        const double rho = sqrt(xi_[5]*(_P.get_X()*_P.get_X()+_P.get_Y()*_P.get_Y())+_P.get_Z()*_P.get_Z());
        const double rho_inv = 1./rho;
        const double nu_inv = 1./(xi_[4]*rho + (1-xi_[4])*_P.get_Z());
        const double nu2rho_inv = rho_inv*nu_inv*nu_inv;

        _J.resize(2,6);
        _J[0][0] = _P.get_X()*nu_inv;                                                                       // du/dpx
        _J[1][1] = _P.get_Y()*nu_inv;                                                                       // dv/dpy
        _J[0][2] = _J[1][3] = 1;                                                                            // du/du0, dv/dv0
        _J[0][4] = xi_[0]*_P.get_X()*(_P.get_Z()-rho)*nu_inv*nu_inv;                                        // du/dalpha
        _J[1][4] = xi_[1]*_P.get_Y()*(_P.get_Z()-rho)*nu_inv*nu_inv;                                        // dv/dalpha
        _J[0][5] = -0.5*xi_[0]*_P.get_X()*xi_[4]*(_P.get_X()*_P.get_X()+_P.get_Y()*_P.get_Y())*nu2rho_inv;  // du/dbeta
        _J[1][5] = -0.5*xi_[1]*_P.get_Y()*xi_[4]*(_P.get_X()*_P.get_X()+_P.get_Y()*_P.get_Y())*nu2rho_inv;  // dv/dbeta
    }


    // write the Jacobian wrt extrinsic parameters
    // J should be 2x6
    void computeJacobianExtrinsic(const vpPoint &_P, vpMatrix &_J)
    {
        const double rho = sqrt(xi_[5]*(_P.get_X()*_P.get_X()+_P.get_Y()*_P.get_Y())+_P.get_Z()*_P.get_Z());
        const double rho_inv = 1./rho;
        const double nu_inv = 1./(xi_[4]*rho + (1-xi_[4])*_P.get_Z());
        const double nu2rho_inv = rho_inv*nu_inv*nu_inv;

        dPdX_[0][0] = xi_[0]*(nu_inv - xi_[4]*xi_[5]*_P.get_X()*_P.get_X()*nu2rho_inv);     // du/dX
        dPdX_[0][1] = -xi_[0]*(xi_[4]*xi_[5]*_P.get_X()*_P.get_Y()*nu2rho_inv);             // du/dY
        dPdX_[0][2] = -xi_[0]*_P.get_X()*nu_inv*nu_inv*(1-xi_[4]+xi_[4]*_P.get_Z()*rho_inv);// du/dZ
        dPdX_[1][0] = -xi_[1]*(xi_[4]*xi_[5]*_P.get_X()*_P.get_Y()*nu2rho_inv);             // dv/dX
        dPdX_[1][1] = xi_[1]*(nu_inv - xi_[4]*xi_[5]*_P.get_Y()*_P.get_Y()*nu2rho_inv);     // dv/dY
        dPdX_[1][2] = -xi_[1]*_P.get_Y()*nu_inv*nu_inv*(1-xi_[4]+xi_[4]*_P.get_Z()*rho_inv);// dv/dZ

        vpFeatureBuilder::create(p_, _P);
        _J = dPdX_ * p_.interaction();
    }

    // update parameter value
    void updateIntrinsic(const vpColVector &_dxi)
    {
        // update
        xi_ += _dxi;

        // all parameters should be positive
        for(int i = 0;i<6;++i)
        {
            if(xi_[i] < 0)
                xi_[i] = 0;
        }
        // alpha should be lesser than 1
        if(xi_[4] > 1)
            xi_[4] = 1;
    }
};

}

#endif // DISTORTION_CAMERA_H
