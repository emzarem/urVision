#include "..\include\PlantTracker.h"

const float X_UNCERTAINTY = 100;
const float RADIUS_UNCERTAINTY = 30;

const float MIN_Y_PER_FRAME = 0;
const float MAX_Y_PER_FRAME = 200;

const float ALLOWABLE_FRAMES_TO_DETECT = 3;

PlantTracker::PlantTracker(int logging) : m_doLogging(logging)
{
	// Set defaults
	m_x_uncertainty = X_UNCERTAINTY;
	m_r_uncertainty = RADIUS_UNCERTAINTY;

	m_min_delta_y = MIN_Y_PER_FRAME;
	m_max_delta_y = MAX_Y_PER_FRAME;
}

PlantTracker::~PlantTracker()
{
}

int PlantTracker::addToTracker(vector<KeyPoint> keypoints)
{
	// Adding new points to tracker, so all zero out newPlants vector before updating it
	m_newPlants.clear();

	// TODO: Increase accuracy by keeping moving average of reasonable y distances
	//vector<float> y_distance_accumulator;

	// Iterate through new objects
	for (vector<KeyPoint>::iterator it = keypoints.begin(); it != keypoints.end(); ++it)
	{
		// Assume there is no match (it is new)
		int foundMatch = false;
		KeyPoint currentPlant = *it;

		vector<KeyPoint>::iterator element_to_erase;
		// Iterate through every old object
		for (vector<KeyPoint>::iterator it2 = m_lastPlants.begin(); it2 != m_lastPlants.end() && false == foundMatch; ++it2)
		{
			KeyPoint lastPlant = *it2;
			// We guarantee that we are moving forward
			if (currentPlant.pt.y - lastPlant.pt.y > -5)
			{
				float x_diff = abs(currentPlant.pt.x - lastPlant.pt.x);
				// If the x values roughly line up AND 
				if (x_diff < m_x_uncertainty)
				{
					float r_diff = abs(currentPlant.size - lastPlant.size);
					// check for radius similarity
					if (r_diff < m_r_uncertainty)
					{
						float y_diff = currentPlant.pt.y - lastPlant.pt.y;
						// Checks for a reasonable amount of frame movement
						if (y_diff < m_max_delta_y && y_diff >=  m_min_delta_y)
						{
							// Check for radius similarity
							// We found a match from the last frame
							foundMatch = true;
							element_to_erase = it2;
						}
					}
				}
			}
		}

		// If no match found, this is a new plant!
		if (false == foundMatch)
		{
			m_newPlants.push_back(currentPlant);
		}
		else m_lastPlants.erase(element_to_erase);
	}

	// Current plants becomes the last plants!
	m_lastPlants = keypoints;

	return true;
}

vector<KeyPoint> PlantTracker::getNewPlants()
{
	return m_newPlants;
}
