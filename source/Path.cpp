
#include "Path.h"
#include "misc/utils.h"

std::list<Vector3> Path::CreateRandomPath(int   NumWaypoints,
                                           double MinX,
                                           double MinY,
                                           double MaxX,
                                           double MaxY)
{
    m_WayPoints.clear();

	//TODO: replace this with n number of random points within given bounds?

	//temp measure
	Vector3 temp1(600.0, 0.0, 600.0);
	Vector3 temp2(600.0, 0.0, -600.0);
	Vector3 temp3(-600.0, 0.0, -600.0);
	Vector3 temp4(-600.0, 0.0, 600.0);

	m_WayPoints.push_back(temp1);
	m_WayPoints.push_back(temp2);
	m_WayPoints.push_back(temp3);
	m_WayPoints.push_back(temp4);

    curWaypoint = m_WayPoints.begin();

    return m_WayPoints;
}


void Path::Render()const
{
	//TODO: use lines (and circles/spheres?) to show path & nodes
}
