import numpy as np
from math import pi, cos, sin, ceil
import matplotlib.pyplot as plt
from scipy.optimize import minimize, Bounds
from copy import copy

class Track( object ):
    '''
    Track object holds all parameters defining the track, as well as the
    constraints under which this track was designed.

    Attributes:
        length      Desired length of track
        rmin        Minimum corner radius
        rmax        Maximum corner radius
        lmax        Maximum straight length
        lmin        Minimum straight length
        dthmax      Maximum angle change of corner
        dthmin      Minimum angle change of corner
        left        Orientation of track (Left-turning if True)
        width       Track width
        crns        Track lay-out
        lpar        Optimized length parameters
        delTh       Optimized angle changes
        optimized   Has this track been optimized yet?
    '''

    def __init__(
        self,
        length = 500.,
        rmin = 9.,
        rmax = 50.,
        lmax = 80.,
        lmin = 5.,
        left = True,
        dthmin = pi/6,
        dthmax = pi,
        width = 3.,
        crns = np.zeros( (0,), dtype=bool ),
        lpar = None,
        delTh = None,
    ):

        self.length = length
        self.rmin   = rmin
        self.rmax   = rmax
        self.lmax   = lmax
        self.lmin   = lmin
        self.dthmax = dthmax
        self.dthmin = dthmin
        self.left   = left # track orientation

        self.width = width

        self.crns  = crns
        self.lpar  = np.zeros( np.shape(crns), dtype=float ) if lpar is None else copy(lpar)
        self.delTh = np.zeros( np.shape(crns), dtype=float ) if delTh is None else copy(delTh)

        # boundaries
        self.xb1 = np.zeros( (0,) )
        self.xb2 = np.zeros( (0,) )
        self.yb1 = np.zeros( (0,) )
        self.yb2 = np.zeros( (0,) )

        # midline
        self.xm = np.zeros( (0,) )
        self.ym = np.zeros( (0,) )
        self.sm = np.zeros( (0,) )
        self.th = np.zeros( (0,) )

        # cones
        self.xc1 = np.zeros( (0,) )
        self.xc2 = np.zeros( (0,) )

        self.optimized = False

    def solve( self, lpar_init, delTh_init, case = 0 ):
        '''
        Solves the optimization problem that ensures the track has the correct
        length, curvature, etc. using an SLSQP algorithm.
        - Case 0: maximizes curvature
        - Case 1: minimizes curvature
        - Case 2: only satisfies constraints
        '''

        nseg = len(lpar_init)
        assert nseg == len( delTh_init )
        assert nseg == len( self.crns )

        if case > 2:
            raise ValueError('Case number higher than 2')

        # Decide on objective function
        if case == 0:
            print("Maximizing curvature of track")
            fobj = objMaxCurv
        elif case == 1:
            print("Minimizing curvature of track")
            fobj = objMinCurv
        elif case == 2:
            print("No minimization")
            fobj = objNone

        x0 = np.hstack( (lpar_init, delTh_init) )

        # check inputs

        # equality constraints
        constr = {}
        constr['type'] = 'eq'
        constr['fun']  = eqConstr
        # constr['jac']  = eqConstrGrad
        constr['args'] = (self.crns,self.length,self.left)

        # bounds
        lb = np.zeros( x0.shape )
        ub = np.zeros( x0.shape )
        for jj in range(0,nseg):
            if self.crns[jj]:
                lb[jj+nseg] = self.dthmin
                ub[jj+nseg] = self.dthmax
                if lpar_init[jj] > 0.:
                    lb[jj] =  self.rmin
                    ub[jj] =  self.rmax
                else:
                    ub[jj] = -self.rmin
                    lb[jj] = -self.rmax
            else:
                lb[jj]      = self.lmin
                lb[jj+nseg] = 0.
                ub[jj]      = self.lmax
                ub[jj+nseg] = 0.

        bnds = Bounds( lb, ub )

        soldict = minimize( fobj, x0, args=(self.lmax,self.dthmax), method='SLSQP',
                            jac=True, bounds=bnds, constraints=constr,
                            tol=None, options=None )

        print(soldict.message)

        self.lpar  = soldict.x[0:nseg]
        self.delTh = soldict.x[nseg:]

        self.optimized = True

        return soldict

    def endpoint( self ):
        '''
        Returns endpoint of the track. If optimization is successful, should be the origin.
        '''
        return compEndpoint( self.crns, self.lpar, self.delTh )

    def plot( self, cones=False, aveDist=3., show = False, filename = "track.png" ):
        '''
        Plots the track defined in the Track object.
        '''

        if np.shape(self.xb1)[0] == 0:
            self.compTrackXY()

        if (np.shape(self.xc1)[0] == 0) and cones:
            self.populateCones( aveDist )

        # actually plot
        plt.figure()
        if self.left:
            plt.fill(self.xb1,self.yb1, '0.75' )
            plt.fill(self.xb2,self.yb2, 'w' )
        else:
            plt.fill(self.xb2,self.yb2, '0.75' )
            plt.fill(self.xb1,self.yb1, 'w' )

        plt.plot(self.xm,self.ym,'k--',linewidth=1)
        plt.plot(self.xb1,self.yb1,linewidth=2,color='k')
        plt.plot(self.xb2,self.yb2,linewidth=2,color='k')

        if cones:
            plt.plot( self.xc1, self.yc1, 'ro' )
            plt.plot( self.xc2, self.yc2, 'go' )

        plt.axis('equal')
        if show:
            plt.show()
        if filename is not None:
            plt.savefig(filename)

    def compTrackXY( self ):
        '''
        Computes track in x,y-space.
        '''

        nplot = 50 # number of points used for corners

        nseg  = len(self.crns)
        ncrns = sum(self.crns)
        npts  = ncrns * nplot + ( nseg - ncrns ) * 2 + 1

        xmid  = np.zeros( (npts,) )
        ymid  = np.zeros( (npts,) )
        smid  = np.zeros( (npts,) )

        theta = np.zeros( (npts,) )

        thcum = 0.

        ind = 0

        for jj in range( 0, nseg ):
            if self.crns[jj]:
                phi = np.linspace( 0., self.delTh[jj], nplot )

                delx =             abs(self.lpar[jj]) * np.sin( phi ) # local coordinate frame
                dely = self.lpar[jj] - self.lpar[jj]  * np.cos( phi ) # local coordinate frame

                # map to global coordinate frame
                xmid[(ind+1):(ind+nplot+1)] = xmid[ind] + delx * cos(thcum) - dely * sin(thcum)
                ymid[(ind+1):(ind+nplot+1)] = ymid[ind] + dely * cos(thcum) + delx * sin(thcum)

                # update cumulative angle
                thcum += np.sign(self.lpar[jj]) * self.delTh[jj]
                theta[(ind+1):(ind+nplot+1)] = theta[ind] + np.sign(self.lpar[jj]) * phi

                # update distance
                smid[(ind+1):(ind+nplot+1)] = smid[ind] + abs(self.lpar[jj]) * phi

                ind += nplot

            else:
                xmid[ind+1] = xmid[ind]
                ymid[ind+1] = ymid[ind]
                smid[ind+1] = smid[ind]

                xmid[ind+2] = xmid[ind] + self.lpar[jj] * cos(thcum)
                ymid[ind+2] = ymid[ind] + self.lpar[jj] * sin(thcum)
                smid[ind+2] = smid[ind] + self.lpar[jj]

                theta[ind+1] = theta[ind]
                theta[ind+2] = theta[ind]

                ind += 2

        self.xb1 = xmid + self.width/2 * np.sin( theta )
        self.yb1 = ymid - self.width/2 * np.cos( theta )
        self.xb2 = xmid - self.width/2 * np.sin( theta )
        self.yb2 = ymid + self.width/2 * np.cos( theta )

        self.xm = xmid
        self.ym = ymid
        self.th = theta
        self.sm = smid

    def populateCones( self, aveDist ):
        '''
            Populates track with cones.
        '''
        # dist is distance between cones as computed from midline

        nseg = len(self.crns)

        xc1 = np.zeros( (0,) )
        yc1 = np.zeros( (0,) )

        xc2 = np.zeros( (0,) )
        yc2 = np.zeros( (0,) )

        thcum = 0.

        for jj in range( 0, nseg ):
            if self.crns[jj]:

                r1 = self.lpar[jj] - self.width/2
                r2 = self.lpar[jj] + self.width/2

                n1 = int( ceil( self.delTh[jj] * abs(r1) / aveDist ) ) + 1 # number of points used on left boundary
                n2 = int( ceil( self.delTh[jj] * abs(r2) / aveDist ) ) + 1 # number of points used on right boundary

                phi1 = np.linspace( 0., self.delTh[jj], n1 )
                phi2 = np.linspace( 0., self.delTh[jj], n2 )

                # delete first point
                phi1 = np.delete( phi1, 0 )
                phi2 = np.delete( phi2, 0 )

                delx1 =  abs(r1) * np.sin( phi1 ) # local coordinate frame
                dely1 = r1 - r1  * np.cos( phi1 ) # local coordinate frame

                delx2 =  abs(r2) * np.sin( phi2 ) # local coordinate frame
                dely2 = r2 - r2  * np.cos( phi2 ) # local coordinate frame

                # map to global coordinate frame
                x1 = delx1 * cos(thcum) - dely1 * sin(thcum)
                y1 = dely1 * cos(thcum) + delx1 * sin(thcum)

                x2 = delx2 * cos(thcum) - dely2 * sin(thcum)
                y2 = dely2 * cos(thcum) + delx2 * sin(thcum)

                if len(xc1) > 0:
                    x1 += xc1[-1]
                    y1 += yc1[-1]
                    x2 += xc2[-1]
                    y2 += yc2[-1]

                # update cumulative angle
                thcum += np.sign(self.lpar[jj]) * self.delTh[jj]

                # append
                xc1 = np.hstack( [xc1, x1] )
                yc1 = np.hstack( [yc1, y1] )
                xc2 = np.hstack( [xc2, x2] )
                yc2 = np.hstack( [yc2, y2] )

            else:

                n = int( ceil( self.lpar[jj] / aveDist ) ) + 1

                xloc = np.linspace( 0, self.lpar[jj], n )
                xloc = np.delete( xloc, 0 )

                x1 = xloc * cos(thcum)
                y1 = xloc * sin(thcum)
                x2 = xloc * cos(thcum)
                y2 = xloc * sin(thcum)

                if len(xc1) > 0:
                    x1 += xc1[-1]
                    y1 += yc1[-1]
                    x2 += xc2[-1]
                    y2 += yc2[-1]
                else:
                    y1 += self.width/2
                    y2 -= self.width/2

                # append
                xc1 = np.hstack( [xc1, x1] )
                yc1 = np.hstack( [yc1, y1] )
                xc2 = np.hstack( [xc2, x2] )
                yc2 = np.hstack( [yc2, y2] )

        self.xc1 = xc1
        self.xc2 = xc2
        self.yc1 = yc1
        self.yc2 = yc2

def eqConstr( x, crns, leng, left ):
    '''
    Computes the value of the equality constraints for `x`.
    '''
    constr = np.zeros( (4,) )

    nseg = int( len(x)/2 )

    # length constraint
    constr[0] = compLength( crns, x[0:nseg], x[nseg:] )
    constr[0] -= leng

    # end point constraints and angle constraint
    constr[1], constr[2], constr[3] = compEndpoint( crns, x[0:nseg], x[nseg:] )
    constr[3] -= (-1 + left*2)*2*pi
    return constr

def compLength( crns, lpar, delTh ):
    '''
    Computes final length of track, defined by corner definition `crns`, length
    parameters `lpar`, and angle changes `delTh`.
    Also computes gradient of length with respect to design variables.
    '''
    trlen = 0.

    for jj in range(0,len(crns)):
        if crns[jj]:
            trlen += abs(lpar[jj]) * delTh[jj]
        else:
            trlen += lpar[jj]

    return trlen

def compEndpoint( crns, lpar, delTh ):
    '''
    Computes end point of track, defined by corner definition `crns`,
    length parameters `lpar`, and angle changes `delTh`.
    Also computes gradient with respect to design variables.
    '''

    xend = 0.
    yend = 0.

    thcum = 0.

    xend  = 0.
    yend  = 0.
    thcum = 0.

    for jj in range(0,len(crns)):
        jjm1 = max(jj-1,0)
        if crns[jj]:
            delx =        abs(lpar[jj]) * sin( delTh[jj] ) # local coordinate frame
            dely = lpar[jj] - lpar[jj]  * cos( delTh[jj] ) # local coordinate frame

            # map to global coordinate frame
            xend += delx * cos(thcum) - dely * sin(thcum)
            yend += dely * cos(thcum) + delx * sin(thcum)

            # update cumulative angle
            thcum += np.sign(lpar[jj]) * delTh[jj]
        else:
            xend += lpar[jj] * cos(thcum)
            yend += lpar[jj] * sin(thcum)

    return xend, yend, thcum

def compCurvature( lpar, delTh, lmax, dthmax ):
    '''
    Computes track curvature and the gradient of track curvature with respect
    to the design variables.
    '''

    curv = ( np.linalg.norm(lpar) / lmax )**2 + \
           ( np.linalg.norm(delTh) / dthmax )**2

    dcurvdlpar  = 2*lpar   / lmax**2
    dcurvddelth = 2*delTh  / dthmax**2

    return curv, dcurvdlpar, dcurvddelth

def objMaxCurv( x, lmax, dthmax ):
    '''
    Objective function for maximum curvature.
    '''
    nseg = int( len( x )/2 )

    c, dcdlpar, dcddelth = compCurvature( x[0:nseg], x[nseg:], lmax, dthmax )

    return -c, np.hstack( ( -dcdlpar, -dcddelth ) )

def objMinCurv( x, lmax, dthmax ):
    '''
    Objective function for minimum curvature.
    '''
    nseg = int( len( x )/2 )

    c, dcdlpar, dcddelth = compCurvature( x[0:nseg], x[nseg:], lmax, dthmax )

    return c, np.hstack( ( dcdlpar, dcddelth ) )

def objNone( x, lmax, dthmax ):
    '''
    Constant objective function.
    '''
    return 1., np.zeros( (len(x),) )
