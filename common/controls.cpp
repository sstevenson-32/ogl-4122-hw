// Include GLFW
#include <GLFW/glfw3.h>
extern GLFWwindow* window; // The "extern" keyword here is to access the variable "window" declared in tutorialXXX.cpp. This is a hack to keep the tutorials simple. Please avoid this.

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include "controls.hpp"

#include <cstdio>

glm::mat4 ViewMatrix;
glm::mat4 ProjectionMatrix;

glm::mat4 getViewMatrix(){
	return ViewMatrix;
}
glm::mat4 getProjectionMatrix(){
	return ProjectionMatrix;
}

int lightingStatus = 1;
int getLightingStatus() {
	return lightingStatus;
}

//Button debouncing for lighting toggle
bool isPressed = false;

// Initial position : on +Z
glm::vec3 position = glm::vec3( 0, 1.5f, 5 ); 
// Initial horizontal angle : toward -Z
float horizontalAngle = 3.14f;
// Initial vertical angle : none
float verticalAngle = 0.0f;
// Initial Field of View
float initialFoV = 45.0f;

float speed = 3.0f; // 3 units / second
float angularSpeed = 3.0f;



void computeMatricesFromInputs(){

	// glfwGetTime is called only once, the first time this function is called
	static double lastTime = glfwGetTime();

	// Compute time difference between current and last frame
	double currentTime = glfwGetTime();
	float deltaTime = float(currentTime - lastTime);
	float radius = glm::sqrt(position.x * position.x + position.y * position.y + position.z * position.z);

	// Direction : Spherical coordinates to Cartesian coordinates conversion
	glm::vec3 direction(
		cos(verticalAngle) * sin(horizontalAngle), 
		sin(verticalAngle),
		cos(verticalAngle) * cos(horizontalAngle)
	);
	
	// Right vector
	glm::vec3 right = glm::vec3(
		sin(horizontalAngle - 3.14f/2.0f), 
		0,
		cos(horizontalAngle - 3.14f/2.0f)
	);
	
	// Up vector
	glm::vec3 up = glm::cross( right, direction );

	// Rotate camera right, maintaining radial distance from origin
	if (glfwGetKey (window, GLFW_KEY_D ) == GLFW_PRESS) {
		//Angle the camera
		horizontalAngle += angularSpeed * deltaTime;
	}
	// Rotate camera left, maintaining radial distance from origin
	if (glfwGetKey (window, GLFW_KEY_A ) == GLFW_PRESS) {
		//Angle the camera
		horizontalAngle -= angularSpeed * deltaTime;
	}

	// Redially rotate the camera up, maintaining distance from origin
	if (glfwGetKey( window, GLFW_KEY_UP ) == GLFW_PRESS) {
		// Angle the camera
		verticalAngle -= angularSpeed * deltaTime;
	}
	// Radially rotate the camera down
	if (glfwGetKey( window, GLFW_KEY_DOWN ) == GLFW_PRESS) {
		// Angle the camera
		verticalAngle += angularSpeed * deltaTime;
	}

	//Set position based on changes to angles
	position = radius * glm::vec3(
        cos(verticalAngle) * sin(horizontalAngle - 3.14f),
        sin(verticalAngle),
        cos(verticalAngle) * cos(horizontalAngle - 3.14f)
    );
	//Set direction so we are always looking at the origin
	direction = -position;

	// Move forward, closer to the origin
	if (glfwGetKey( window, GLFW_KEY_W ) == GLFW_PRESS) {
		position += direction * deltaTime * speed;
	}
	// Move backward, away from the origin
	if (glfwGetKey( window, GLFW_KEY_S ) == GLFW_PRESS) {
		position -= direction * deltaTime * speed;
	}

	//Toggle lighting if keypress is new
	if (glfwGetKey( window, GLFW_KEY_L ) == GLFW_PRESS) {
		//Only change lighting status if was not pressed in the last cycle
		if (!isPressed) {
			lightingStatus = (lightingStatus + 1) % 2;
			isPressed = true;
		}
	} else {
		//No longer pressed, toggle state
		isPressed = false;
	}

	// printf("Position: (%f, %f, %f). Radius: (%f). horizontalAngle: (%f). verticleAngle: (%f).\n", position.x, position.y, position.z, radius, horizontalAngle, verticalAngle);

	// Projection matrix : 45� Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
	ProjectionMatrix = glm::perspective(glm::radians(initialFoV), 4.0f / 3.0f, 0.1f, 100.0f);
	// Camera matrix
	ViewMatrix       = glm::lookAt(
								position,           // Camera is here
								position+direction, // and looks here : at the same position, plus "direction"
								up                  // Head is up (set to 0,-1,0 to look upside-down)
						   );

	// For the next frame, the "last time" will be "now"
	lastTime = currentTime;
}