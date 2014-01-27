#!/bin/sh

cp Gualzru.mesh  /usr/local/share/gazebo/Media/models/
cp gualzru.model /usr/local/share/gazebo/Media/models/
cp robolab.png /usr/local/share/gazebo/Media/materials/textures/

echo "
material Gazebo/RoboLab
{
	receive_shadows on
	technique
	{
		pass
		{
			ambient 1.0 1.0 1.0 1.0
			diffuse 1.0 1.0 1.0 1.0
			specular 0.0 0.0 0.0 1.0 12.5

			texture_unit
			{
				texture robolab.png
				filtering trilinear
			}
		}
	}
}
" >> /usr/local/share/gazebo/Media/materials/scripts/Gazebo.material
