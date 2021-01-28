#!/usr/bin/env python  
import rospy
import datetime
import random
from geometry_msgs.msg import Point, PointStamped
from math import sin,cos,sqrt,acos,pi
from numpy import deg2rad
import numpy as np

def to_odom_frame(p, latitude,longitude,r):
    phi = deg2rad(90-latitude)
    theta = deg2rad(longitude)
    #the x axis of the /odom frame is defined by the derivative of r_hat with respect to theta.
    xhat = np.array((-r*sin(theta)*sin(phi), r*cos(theta)*sin(phi),0))
    #the y axis of the /odom frame is defined by the derivative of r_hat with respect to phi
    yhat = np.array((r*cos(theta)*cos(phi), r*sin(theta)*cos(phi),-r*sin(phi)))
    #the z axis of the /odom frame is defined by the  r_hat vector itself
    zhat = np.array((r*cos(theta)*sin(phi), r*sin(theta)*sin(phi), r*cos(phi)))

    #normalize to make them unit vectors

    #handle special case at the poles
    if(np.sqrt(np.sum(xhat**2)) == 0):
        xhat = np.cross(yhat, zhat)

    xhat = xhat / np.sqrt(np.sum(xhat**2))
    yhat = yhat / np.sqrt(np.sum(yhat**2))
    zhat = zhat / np.sqrt(np.sum(zhat**2))
    point = np.array(p)

    #r_hat = np.array((r*cos(theta)*sin(phi), r*sin(theta)*sin(phi), r*cos(phi)))
    trans =  np.array((0,0,r))
    #m = np.vstack((np.array([xhat,yhat,zhat,r_hat]).T, [0,0,0,1]))
    m = np.array([xhat,yhat,zhat]).T

    return np.dot(point,m)[:3] - trans

def publish_polaris_coords():

    """
        This node publishes the coordinates of the projection of polaris on Earth now.
        More specifically, the 3D Cartesian coordinates x,y,z in the /odom frame of the point
        defined by the intersection between the surface of the Earth and the line between polaris and the center of the Earth.
    """

    pub            = rospy.Publisher('/polaris_odom_coords',PointStamped, queue_size=20)
    rate           = rospy.Rate(100)

    radius    = rospy.get_param("earth_radius")
    latitude  = rospy.get_param("odom_latitude")
    longitude = rospy.get_param("odom_longitude")

    #polaris celestial coordinates
    polaris_ra     = rospy.get_param('polaris_ra')
    polaris_dec    = rospy.get_param('polaris_dec')

    while not rospy.is_shutdown():
        """
            the number of hours polaris is behind the sun, i.e. how many hours before/after
            the sun Polaris rises above the horizon, let's call this number - H,
            depends on the time of the year.
            RA represents the number of hours Polaris is behind the sun on march 21,
            So, if we stand on meridian 0 at 12 noon on march 21, polaris will be above meridian RA*15
`           (360 degrees /24 hours = 15 degrees/hour)


            for the purposes of this exercise we will assume the orbit of the Earth is a circle,
            which makes the rate of change in H through the year constant at 24/365 and simplifies the math a little.
        """
        utc_now = datetime.datetime.utcnow()
        equinox = datetime.datetime(utc_now.year,3,21,12,0,0)

        #time since equinox
        tse = utc_now - equinox
        days_since_equinox = tse.days + tse.seconds/(24.0*60*60)

        #for the purpose of this exercise we will not worry about leap years
        hours_behind_the_sun = polaris_ra + days_since_equinox*24.0/365.0
        hours_behind_the_sun += 24
        hours_behind_the_sun %= 24
        meridian_at_12_utc = hours_behind_the_sun*15

        hours_past_noon = utc_now.hour-12 + utc_now.minute/60.0 + utc_now.second/(60*60.0)

        #here we switch to spherical coordinates 
        theta_p    = meridian_at_12_utc - hours_past_noon*15
        theta_p    += 360
        theta_p    %= 360
        theta_p    *= (pi/180)
        phi_p   = (pi/180)*(90 - polaris_dec)

        rospy.logdebug("THETA_P :%s"%(theta_p*180/pi))
        rospy.logdebug("PHI_P :%s"%(phi_p*180/pi))

        x_p = radius*cos(theta_p)*sin(phi_p)
        y_p = radius*sin(theta_p)*sin(phi_p)
        z_p = radius*cos(phi_p)

        p = to_odom_frame([x_p,y_p,z_p],latitude,longitude, radius)
        ps = PointStamped()
        ps.header.frame_id = 'odom'
        ps.header.stamp = rospy.Time.now()
        ps.point = Point(*p)
        pub.publish(ps)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('polaris_coords_publisher',log_level=rospy.DEBUG)
    try:
        print('starting')
        publish_polaris_coords()
    except rospy.ROSInterruptException:
        pass
