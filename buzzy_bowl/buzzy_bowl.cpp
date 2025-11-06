/*
Author: Samir Stevenson, Aaron Marlin
Class: ECE4122
Last Date Modified: 

Description: 
*/

/*
Build Instructions
mkdir build; cd ./build
cmake ../
cmake --build . --target tutorial09_several_objects
*/

// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <vector>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>
GLFWwindow* window;

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include <common/shader.hpp>
#include <common/texture.hpp>
#include <common/controls.hpp>
#include <common/objloader.hpp>
#include <common/vboindexer.hpp>

int main( void )
{
	// 1) Initialize GLFW - Windowing API enabling OpenGL to display results
	if( !glfwInit() )
	{
		fprintf( stderr, "Failed to initialize GLFW\n" );
		getchar();
		return -1;
	}

	// 2) Set profile hints, requesting OpenGL3.3 core context with 4x MSAA
	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make macOS happy; should not be needed
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// 3) Open a window and create its OpenGL context
	window = glfwCreateWindow( 1024, 768, "Tutorial 09 - Rendering several models", NULL, NULL);
	if( window == NULL ){
		fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" );
		getchar();
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
    
	// 4) Initialize GLEW - Load OpenGL function pointers from graphics driver
	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		getchar();
		glfwTerminate();
		return -1;
	}

	// 5) Configure input
	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

	// 6) Set basic GL state
	// Dark blue background
	glClearColor(0.0f, 0.0f, 0.4f, 0.0f);
	// Enable depth test - Closer fragments are shown
	glEnable(GL_DEPTH_TEST);
	// Accept fragment if it is closer to the camera than the former one
	glDepthFunc(GL_LESS); 
	// Cull triangles which normal is not towards the camera - Do not process triangles we can't see
	glEnable(GL_CULL_FACE);

	// 7) Create and bind a Vertax Array Object (VAO) - Configuration of a single piece of geometry
	GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);

	// 8) Compile and use shaders
	// Create and compile our GLSL program from the shaders
	GLuint programID = LoadShaders( "StandardShading.vertexshader", "StandardShading.fragmentshader" );
	// Get a handle for our "MVP" uniform - Index the GPU uses to access this variable
	GLuint MatrixID = glGetUniformLocation(programID, "MVP");
	GLuint ViewMatrixID = glGetUniformLocation(programID, "V");
	GLuint ModelMatrixID = glGetUniformLocation(programID, "M");

	// 9) Load texture and cache its location on the GPU
	// Load the textures
	GLuint Texture = loadDDS("uvmap.DDS");
	//GLuint RectTexture = loadDDS("greenTile.DDS");
	GLuint RectTexture = loadBMP_custom("ff.bmp");
	// Get a handle for our "myTextureSampler" uniform
	GLuint TextureID  = glGetUniformLocation(programID, "myTextureSampler");


	// 10) Load and index model (OBJ) - Read into CPU memory and create indexed arrays, enabling shared verticies
	// Read our .obj file
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec2> uvs;
	std::vector<glm::vec3> normals;
	bool res = loadOBJ("suzanne.obj", vertices, uvs, normals);

	std::vector<unsigned short> indices;
	std::vector<glm::vec3> indexed_vertices;
	std::vector<glm::vec2> indexed_uvs;
	std::vector<glm::vec3> indexed_normals;
	indexVBO(vertices, uvs, normals, indices, indexed_vertices, indexed_uvs, indexed_normals);

	// 11) Create and upload VBOs and index buffer - Loading .obj into GPU memory
	// Load it into a VBO - Vertex data stored in GPU memory
	GLuint vertexbuffer;
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, indexed_vertices.size() * sizeof(glm::vec3), &indexed_vertices[0], GL_STATIC_DRAW);

	GLuint uvbuffer;
	glGenBuffers(1, &uvbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
	glBufferData(GL_ARRAY_BUFFER, indexed_uvs.size() * sizeof(glm::vec2), &indexed_uvs[0], GL_STATIC_DRAW);

	GLuint normalbuffer;
	glGenBuffers(1, &normalbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
	glBufferData(GL_ARRAY_BUFFER, indexed_normals.size() * sizeof(glm::vec3), &indexed_normals[0], GL_STATIC_DRAW);

	// Generate a buffer for the indices as well
	GLuint elementbuffer;
	glGenBuffers(1, &elementbuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned short), &indices[0] , GL_STATIC_DRAW);

	//Create a uniform to control lighting
	GLuint lightingUniform = glGetUniformLocation(programID, "lightingOpts");
	glUniform1i(lightingUniform, 1);

	// 12) Add our rectangle
	float rectSize = 4.0f;
	float rectWidth = 50.0f;
	float rectHeight = 30.0f;
	float yPos = 0.0f;
	// Set base vertices
	static const GLfloat rectVertexData[] = {
		-rectHeight, yPos,  rectWidth,
		rectHeight,  yPos,  rectWidth,
		-rectHeight, yPos, rectWidth,

		-rectHeight, yPos, -rectWidth,
		rectHeight,  yPos, -rectWidth,
		rectHeight,  yPos,  rectWidth,
	};
	GLuint rectVertexBuffer;
	glGenBuffers(1, &rectVertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, rectVertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(rectVertexData), rectVertexData, GL_STATIC_DRAW);

	// Set normal vectors for each vertex, each along y axis
	static const GLfloat rectNormalData[] = {
	0,1,0,  0,1,0,  0,1,0,
	0,1,0,  0,1,0,  0,1,0
	};
	GLuint rectNormalBuffer;
	glGenBuffers(1, &rectNormalBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, rectNormalBuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(rectNormalData), rectNormalData, GL_STATIC_DRAW);

	// Set UV vectors for each vertex, determines how a texture is placed on the shape at each vertex
	static const GLfloat rectUVData[] = {
	0,0,  1,1,  0,1,
	0,0,  1,0,  1,1
	};
	GLuint rectUVBuffer;
	glGenBuffers(1, &rectUVBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, rectUVBuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(rectUVData), rectUVData, GL_STATIC_DRAW);

	//Create uniform color shader for the rectangle
	GLuint colorUniform = glGetUniformLocation(programID, "uColor");
	glUseProgram(programID);
	glUniform3f(colorUniform, 0.0f, 1.0f, 0.0f);

	// 13) Cache light uniform and initialize timer - Add light to shader (programID)
	// Get a handle for our "LightPosition" uniform
	glUseProgram(programID);
	GLuint LightID = glGetUniformLocation(programID, "LightPosition_worldspace");

	// For speed computation
	double lastTime = glfwGetTime();
	int nbFrames = 0;

	do{
		// Measure speed
		double currentTime = glfwGetTime();
		nbFrames++;
		if ( currentTime - lastTime >= 1.0 ){ // If last prinf() was more than 1sec ago
			// printf and reset
			printf("%f ms/frame\n", 1000.0/double(nbFrames));
			nbFrames = 0;
			lastTime += 1.0;
		}

		// Clear the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// 1) Compute the MVP matrix from keyboard and mouse input
		computeMatricesFromInputs();
		glm::mat4 ProjectionMatrix = getProjectionMatrix();
		glm::mat4 ViewMatrix = getViewMatrix();

		// 3) Use our shader and set globals to be used by all objects
		// Set light position and view matrix once before drawing objects
		glUseProgram(programID);
		glm::vec3 lightPos = glm::vec3(4,4,4);
		glUniform3f(LightID, lightPos.x, lightPos.y, lightPos.z);
		glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]); // This one doesn't change between objects, so this can be done once for all objects that use "programID"

		// 4) Bind the right texture for the shader to use
		glBindTexture(GL_TEXTURE_2D, Texture);

		// 2) Render our shapes
		for (float angleDegrees = 0.0f; angleDegrees <= 360.0f; angleDegrees += 45.0f) {
			// 1) Set Model and MVP matrices
			glm::mat4 ModelMatrix = glm::mat4(1.0);
			float angle = glm::radians(angleDegrees);
			ModelMatrix = glm::translate(ModelMatrix, glm::vec3(3.5*glm::sin(angle), 1.0f, 3.5*glm::cos(angle)));
			ModelMatrix = glm::rotate(ModelMatrix, angle, glm::vec3(0.0f, 1.0f, 0.0f));
			glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;

			// Send our transformation to the currently bound shader, in the "MVP" uniform
			glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
			glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);

			// 1st attribute buffer : vertices
			glEnableVertexAttribArray(0);
			glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

			// 2nd attribute buffer : UVs
			glEnableVertexAttribArray(1);
			glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
			glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);

			// 3rd attribute buffer : normals
			glEnableVertexAttribArray(2);
			glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
			glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

			// 2) Index buffer
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);

			// 3) Draw the triangles !
			glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_SHORT, (void*)0);
		};

		// 5) Render the rectangle
		{
			// 1) Set uColor to green
			glUseProgram(programID);
			glUniform3f(colorUniform, 0.0f, 1.0f, 0.0f);

			// 2) Bind the texture for our rectangle
			glBindTexture(GL_TEXTURE_2D, RectTexture);

			// 3) Set Model and MVP matrices - Set position to the origin
			glm::mat4 ModelMatrixRect = glm::mat4(1.0);
			glm::mat4 MVPRect = ProjectionMatrix * ViewMatrix * ModelMatrixRect;
			glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVPRect[0][0]);
			glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrixRect[0][0]);

			// 4) Enable position (0)
			glEnableVertexAttribArray(0);
			glBindBuffer(GL_ARRAY_BUFFER, rectVertexBuffer);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

			// 5) Enable UV (1)
			glEnableVertexAttribArray(1);
			glBindBuffer(GL_ARRAY_BUFFER, rectNormalBuffer);
			glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);

			// 6) Enable normals (2)
			glEnableVertexAttribArray(2);
			glBindBuffer(GL_ARRAY_BUFFER, rectUVBuffer);
			glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

			// 7) Draw it, disabling CULL_FACE when drawing
			glDisable(GL_CULL_FACE);
			glDrawArrays(GL_TRIANGLES, 0, 6);
			glEnable(GL_CULL_FACE);
		}

		// 6) Update lighting as needed based on user input
		glUniform1i(lightingUniform, getLightingStatus());

		// 7) Disable vertex attribute arrays (vertices, UVs, normals)
		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);
		glDisableVertexAttribArray(2);

		// 8) Swap buffers - While front buffer is displayed, a back buffer is processed. 
		// This swaps that processed back buffer to the front.
		glfwSwapBuffers(window);
		glfwPollEvents();

	} // Check if the ESC key was pressed or the window was closed
	while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
		   glfwWindowShouldClose(window) == 0 );

	// Cleanup VBO and shader
	glDeleteBuffers(1, &vertexbuffer);
	glDeleteBuffers(1, &uvbuffer);
	glDeleteBuffers(1, &normalbuffer);
	glDeleteBuffers(1, &elementbuffer);
	glDeleteProgram(programID);
	glDeleteTextures(1, &Texture);
	glDeleteVertexArrays(1, &VertexArrayID);

	// Close OpenGL window and terminate GLFW
	glfwTerminate();

	return 0;
}

