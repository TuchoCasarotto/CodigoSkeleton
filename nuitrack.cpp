
#include "nuitrack.h"

#include <string>
#include <vector>
// cálculo de tiempos
#include <time.h>
#include <iostream>
//#include <iomanip>	//set precision


//#include <\marcos\User\funciones.h>

// ***** **** ****


//----------FLAVIO-----------------
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glfw3.lib")


#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#include <math.h>

// About Desktop OpenGL function loaders:
//  Modern desktop OpenGL doesn't have a standard portable header file to load OpenGL function pointers.
//  Helper libraries are often used for this purpose! Here we are supporting a few common ones (gl3w, glew, glad).
//  You may use another loader/header of your choice (glext, glLoadGen, etc.), or chose to manually implement your own.
#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
#include <GL/gl3w.h>    // Initialize with gl3wInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
#include <GL/glew.h>    // Initialize with glewInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
#include <glad/glad.h>  // Initialize with gladLoadGL()
#else
#include IMGUI_IMPL_OPENGL_LOADER_CUSTOM
#endif

// Include glfw3.h after our OpenGL definitions
#include <GLFW/glfw3.h>

// [Win32] Our example includes a copy of glfw3.lib pre-compiled with VS2010 to maximize ease of testing and compatibility with old VS compilers.
// To link with VS2010-era libraries, VS2015+ requires linking with legacy_stdio_definitions.lib, which we do using this pragma.
// Your own project should not be affected, as you are likely to link with a newer binary of GLFW that is adequate for your version of Visual Studio.
#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

static void glfw_error_callback(int error, const char* description)
{
	fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

//----------END FLAVIO----------

//		declaración de funciones nuestras
//		dibuja un huevo entre dos articulaciones
void			Hueso(cv::Mat esqueleto, cv::Point arts[25], int joint1, int joint2, cv::Scalar color, int ancho);
double			Arco(cv::Mat, int joint1, int joint2, int joint3, cv::Scalar, int ancho);
double			filtroAB(double x_obs, double x_old, double &vx_old);
//
double			Ver360(double);
double			RedonD(double aux);
int				RedonI(int aux);

static void		HelpMarker(const char* desc);
void			limpiar_records(void);




//  cv::Point articulacion[42]; B G R
cv::Scalar _cyan	= cv::Scalar(222, 222, 0);
cv::Scalar _green	= cv::Scalar(0, 255, 0);
cv::Scalar _red		= cv::Scalar(0, 0, 255);
cv::Scalar _yellow	= cv::Scalar(0, 255, 255);
cv::Scalar _pink	= cv::Scalar(255, 0, 255);
cv::Scalar _black	= cv::Scalar(0, 0, 0);
cv::Scalar _white	= cv::Scalar(255, 255, 255);
cv::Scalar _fucsia	= cv::Scalar(255, 50, 220);
cv::Scalar _fluo	= cv::Scalar(50, 255, 80);
cv::Scalar _orange	= cv::Scalar(0, 170, 255);

// arrary de puntos para cada joint
tdv::nuitrack::Joint	artFULL[25];					//  las 25 articulaciones completas 
cv::Point				articulacion[25];				//  articulaciones en (x,y)//  
//int						altura[6];					//  altura de cada usuario
//int						alturaMM[6];				//  altura de cada usuario
//int						distancia[6];				//	distancia de cada usuario
int						altura;							//  altura del usuario en pixels
int						alturaMM;						//  altura del usuario en MM
int						alturaMM_2;						//  altura del usuario en MM

int						envergadura;
int						envergaduraMM;

int						distancia;						//	distancia del usuario en MM
//uint16_t				art_depth[25];					//  profundidad de cada articulación
int						cont			= 0;			//  contador
int						Poffset			= 848;			//  848 cuánto desplazo todo para mostrar datos
bool					PROCESANDO		= false;		//	inicio del procesamiento!
int						repeticiones	= 0;			//	contador de repeticiones
int						FLEX_biceps_l	= 0;			//	secuencia de flexión de biceps izquierdo
std::string				out;
double					dist, escala;
int						FPSrealsense	= 60;     
int						real_Dfps		= 90;

//  medición de tiempo
double					T0 = 1 / 30;
clock_t					t_start, t_end;					//	para inicio de aplicación y medición de muestreo
clock_t					t_cur;							//	para alguna tarea
int						t_taken		= 0;
int						t_takenm	= 2000;
int						t_takenM	= 0;
int						conta		= 0;
bool					t_INIT		= true;
bool					CONTANDO	= false;
bool					refresh		= true;				//  para que no muestre a 60Hz
double					tiempo_max	= 0.0;				//	tiempo de ejecución del programa
double					tiempo_cur	= 0.0;				//	tiempo de la tarea

//  medición de ángulos
double		ang_l_codo, ang_r_codo, ang_l_hombro, ang_r_hombro;
//uint16_t	distance;

//	bandera principales

//	ACTIVIDADES ACTUALES: constantes de cada tarea
const int	SALTO_VERTICAL		= 0;		// 
const int	SALTO_LARGO			= 1;		//
const int	JUEGOS				= 2;		// es la forma de seleccionar cuál está activo
const int	POSTURAS			= 3;		// para detectar posturas
const int	FLEXIONES			= 4;		// contador de flexiones con una mano


//	FIJAR ACTIVIDAD INICIAL
int			ACTIVIDADelegida	= 2;		//	0 vertical 1 largo 2 juegos
int			ACTIVIDADprevia		= 0;		//

bool		AYUDA_activado		= true;		//	ventanida de ayuda arriba a la izquierda
bool		VENTANAinteractiva	= false;	//	ventana interactiva del juego en la mano izquierda

//	variables del juego
int			MENUmano			= 0;		//	radio buttons del menú de mano, que se deben corresponder con el radio button de funciones
bool		MANOinicio			= true;		//	para saber si es la primera vez que entra, y se actualiza cada vez que se mueve
ImVec2		MANOinicial;					//	la idea es ir seleccionando
ImVec2		MANOactual;						//	posición actual mano izq
//	arrastra circulito
int			CIRCULOcont			= -1;
int			CIRCULO2cont		= -1;
cv::Point	centro;
cv::Point	centro2;


//	medición del salto vertical
bool		SALTANDO			= false;	//
bool		SALTOdistOK			= false;	//
bool		SALTOpisoOK			= false;	//
int			SALTOindex			= 0;		//	índice del salto
int			SALTOfin			= -1;		//	largo del salto (temporal, número de muestras del salto)
int			SALTO				= 0;		//	salto en mm
int			SALTOlargoMM		= 0;		//	MM del salto en largo
int			SALTOrecord			= 0;		//	mejor salto en mm
double		SALTOtiempo			= 0.0;		//	tiempo de vuelo del salto en s
double		TIEMPOrecord		= 0.0;		//	tiempo de vuelo del mejor salto
int			SALTOcont			= 0;		//
int			SALTOcontadorS		= 0;		//	contador de permanencia
float		SALTOperfil[120];				//
int			CMsaltoLargoX[100];				//	evolución del CM DURANTE EL SALTO EN LARGO
int			CMsaltoLargoY[100];				//
int			SALTOstatus			= 0;		//	estado del salto

//int		SALTObuffer[10];			//
//int		SALTOdist			= 0;	//
//int		SALTOmin			= 100;	//	salto minimo para contarlos en mm
//int		SALTOmax			= 0;	//
//int		SALTOinicioCont		= 0;	//
//cv::Point	SALTOcadera17;
//cv::Point	SALTOcadera21;
//cv::Point	SALTOpies19;
//cv::Point	SALTOpies23;
//cv::Point	SALTOtechoPXa;
//cv::Point	SALTOtechoPXb;
//cv::Point	SALTOhombro06Max;
//cv::Point	SALTOhombro12Max;
//cv::Point	SALTOcadera17Max;
//cv::Point	SALTOcadera21Max;
//cv::Point	SALTOpies19Max;
//cv::Point	SALTOpies23Max;
//int			SALTOhombroMax		= 2000;
//int			SALTOcaderaMax		= 2000;
//int			SALTOpiesMax		= 2000;
//int			SALTOhombroMaxMM;				//	valores maximos en MM
//int			SALTOcaderaMaxMM;
//int			SALTOwaistMaxMM;
//int			SALTOpiesMaxMM;
//int			SALTOhombroBasMM;			//
//int			SALTOcaderaBasMM;
//int			WAISTmaxMM;						//	mas el umbral
//int			SALTOpiesBasMM;
//int			ALTURAhombroMM;
//int			ALTURAcaderaMM;
//int			ALTURAwaistMM;
//int			ALTURApiesMM;
//int			SALTOcontHOMBRO;
//int			SALTOcontCADERA;
//int			SALTOcontWAIST;
//int			SALTOcontPIES;
//int			SALTOcurrentHOMBRO;
//int			SALTOcurrentCADERA;


float		saltos[17];						//	registro de 40 saltos
float		tiempos[17];					//	registro de 40 tiempos para los 40 saltos anteriores
cv::Point	SALTOhombro06;					//
cv::Point	SALTOhombro12;					//
cv::Point	SALTOwaist04a;					//
cv::Point	SALTOwaist04b;					//

cv::Point	SALTOpisoPXa;					//
cv::Point	SALTOpisoPXb;					//

cv::Point	SALTOwaist04aMax;				//	
cv::Point	SALTOwaist04bMax;				//

int			SALTOwaistMaxPX		= 2000;		//	salto máximo WAIST en PIXELS para marcar la línea
int			SALTOwaistBasMM;				//	base fija de la cintura en MM, el valor para medir el salto
int			WAISTminMM;						//	base de cintura menos el umbral para medir la sentadilla
int			WAISTdistanceMM;				//	distancia a la que está la cintura cuando se fija
int			SALTOcurrentWAISTmm;			//	altura del salto neto en mm

//	variables para salto en largo
cv::Point	userTopLeft;					//	rectángulo de usuario
cv::Point	userTopRight;					//	
cv::Point	userBottomLeft;					//	
cv::Point	userBottomRight;				//	

int			mainCMxMM;						//	media de 3 medidas del CM en MM (EJE X)
int			mainCMyMM;						//
int			userCMx_MM;						//	centro de masa x MM
int			userCMy_MM;						//	centro de masa y MM
int			userCMz_MM;						//	centro de masa z MM

int			userCMx_PX;						//	centro de masa x píxels
int			userCMy_PX;						//	centro de masa y píxels
int			userCMx_MM1;					//	centro de masa x píxels AUX 1
int			userCMy_MM1;					//	centro de masa y píxels AUX 1 
int			userCMx_MM2;					//	centro de masa x píxels AUX 2
int			userCMy_MM2;					//	centro de masa y píxels AUX 2 
int			userCMx_MM3;					//	centro de masa x píxels AUX 3
int			userCMy_MM3;					//	centro de masa y píxels AUX 3 

cv::Point	CM_inicial_PX;
cv::Point	CM_inicial_MM;
cv::Point	PUNTAS_inicial_PX;
cv::Point	CM_final_PX;
cv::Point	CM_final_MM;
cv::Point	PUNTAS_final_PX;


//	filtrado general
double const	LAMBDA			= .01;	//AHORA EN FUNCIONES.H
double const	ALFA			= -(LAMBDA*LAMBDA + 8 * LAMBDA - (LAMBDA + 4)*sqrt(LAMBDA*LAMBDA + 8 * LAMBDA)) / 8;	//AHORA EN FUNCIONES.H
double const	BETA			= (LAMBDA*LAMBDA + 4 * LAMBDA - LAMBDA * sqrt(LAMBDA*LAMBDA + 8 * LAMBDA)) / 4;			//AHORA EN FUNCIONES.H
float			x_old			= 0.0;
float			vx_old			= 0.0;
//	cada señal que filtro...
int				dist_aux		= 0;	// DISTANCIA AL USER
int 			distancia_old	= 0; 
double			distanciaP_old	= 0.0;
int				altura_aux		= 0;	//	ALTURA DEL USER
int 			altura_old		= 0;
double			alturaP_old		= 0.0;
int				envergadura_aux = 0;	//	ENVERGADURA DEL USER
int 			envergadura_old = 0;
double			envergadurP_old = 0.0;
double			PISOaux			= 0;	//	PISO ACTUAL DEL USER
double 			PISOaux_old		= 0;
double			PISOauxP_old	= 0.0;


// FPS
//std::string	real_Dfps, real_Rfps;
// ***** **** ****


// Constructor
NuiTrack::NuiTrack( const std::string& config_json )
{
    // Initialize
    initialize( config_json );
}

// Destructor
NuiTrack::~NuiTrack()
{
    // Finalize
    finalize();
}

// =========================================================================================================
// =========================================================================================================
//		funcion de procesamiento principal y GUI
// Processing
int NuiTrack::run()
{

	// Run NuiTrack
	tdv::nuitrack::Nuitrack::run();

	//-----------FLAVIO-----------------
	// Setup window
	glfwSetErrorCallback(glfw_error_callback);
	if (!glfwInit())
		return 1;

	// Decide GL+GLSL versions
#if __APPLE__
	// GL 3.2 + GLSL 150
	const char* glsl_version = "#version 150";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
	// GL 3.0 + GLSL 130
	const char* glsl_version = "#version 130";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	//glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
	//glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif
	// Create window with graphics context
	GLFWwindow* window = glfwCreateWindow(1280, 720, "INAUT PDTS 2019: Deteccion y medicion de movimiento", NULL, NULL);
	glfwMaximizeWindow(window);
	if (window == NULL)
		return 1;
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1); // Enable vsync

	// Initialize OpenGL loader
#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
	bool err = gl3wInit() != 0;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
	bool err = glewInit() != GLEW_OK;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
	bool err = gladLoadGL() == 0;
#else
	bool err = false; // If you use IMGUI_IMPL_OPENGL_LOADER_CUSTOM, your loader is likely to requires some form of initialization.
#endif
	if (err)
	{
		fprintf(stderr, "Failed to initialize OpenGL loader!\n");
		return 1;
	}

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsClassic();
	//ImGui::StyleColorsLight();

	// Setup Platform/Renderer bindings
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init(glsl_version);

	// Load Fonts
	// - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
	// - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
	// - If the file cannot be loaded, the function will return NULL. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
	// - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
	// - Read 'docs/FONTS.txt' for more instructions and details.
	// - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
	//io.Fonts->AddFontDefault();
	//io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
	//io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
	//io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
	//io.Fonts->AddFontFromFileTTF("../../misc/fonts/ProggyTiny.ttf", 10.0f);
	//ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, NULL, io.Fonts->GetGlyphRangesJapanese());
	//IM_ASSERT(font != NULL);

	//	tamaños de fuentes disponibles para la GUI (se pueden agregar más...)
	ImFont* font20 = io.Fonts->AddFontFromFileTTF("../imgui/fonts/Roboto-Medium.ttf", 20.0f);
	ImFont* font24 = io.Fonts->AddFontFromFileTTF("../imgui/fonts/Roboto-Medium.ttf", 24.0f);
	ImFont* font28 = io.Fonts->AddFontFromFileTTF("../imgui/fonts/Roboto-Medium.ttf", 28.0f);
	ImFont* font40 = io.Fonts->AddFontFromFileTTF("../imgui/fonts/Roboto-Medium.ttf", 40.0f);

	// VENTANAS VISIBLES DE LA GUI
	bool	show_demo_window	= false;	//	check del demo
	bool	show_status_window	= true;		//	check de la base de datos
	
	bool    altura_validada		= false;	//	check de altura
	bool    alcance_validado	= false;	//	check de alcance
	bool    envergadura_valid	= false;	//	check de envergadura

	//	colores
	ImVec4  clear_color			= ImVec4(0.45f, 0.55f, 0.60f, 1.00f);		// R G B transparencia --> amarillo a pleno


	//	base de datos
	char	usuario_nombre[51]	= "";
	char	usuario_dni[9]		= "";
	char	usuario_peso[4]		= "";

	//float altura				= 1.85f;
	float	alcance				= 2.35f;
	float	envergadura			= 1.15f;
	//float distancia			= 2.25;
	int		alturaOK;
	int		alcanceOK;
	int		envergaduraOK;
	

	//creo textura para mostrar en la pantalla
	glEnable(GL_TEXTURE_2D);
	GLuint textureTrash;
	glGenTextures(1, &textureTrash);
	glBindTexture(GL_TEXTURE_2D, textureTrash);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	while (!glfwWindowShouldClose(window))
	{
		//		ES EL LOOP PRINCIPAL DE REFRESCO DE LA PANTALLA
		// Update Data
		update();
		// Draw Data
		draw();

		// Poll and handle events (inputs, window resize, etc.)
		// You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
		// - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
		// - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
		// Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
					
		glfwPollEvents();

		// Start the Dear ImGui frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
		

		// **********************************************************************************
		//		VENTANA DE LA DERECHA CON TIEMPOS Y DATOS GENERALES
		//
		ImGui::SetNextWindowPos(ImVec2(1290, 5), ImGuiCond_Always);
		ImGui::SetNextWindowSize(ImVec2(610, 975), ImGuiCond_Always);

		//  título
		ImGui::Begin("Datos del Ejercicio");                    // 
		//	tiempo total
		tiempo_max = (t_start / double(CLOCKS_PER_SEC));
		ImGui::PushFont(font24);								//  cambio a 24
		ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 1.0f), "Tiempo: %07.2f", tiempo_max);	//	
				//	cronómetro
		if (CONTANDO == true) {
			tiempo_cur = (((double)(clock()) - (double)(t_cur)) / double(CLOCKS_PER_SEC));
		}				
		ImGui::SameLine();
		ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 0.8f), "    Crono: %07.2f", tiempo_cur);
		/*
		//	si hace click arranca
		if (ImGui::IsItemClicked()) {
			if (CONTANDO == false) {
				CONTANDO	= true;
				t_cur		= clock();							//	sin mostrarlo, pero RESETEO TEMPORIZADOR
			}
			else {
				CONTANDO = false;
			}
		}
		*/
		ImGui::PopFont();										//  vuelvo a 20
		ImGui::Separator();

																//  vuelvo a 20
		//  ahora quiero meter los características
		ImGui::Checkbox("Estado", &show_status_window);
		ImGui::SameLine();
		//ImGui::Checkbox("Demo Window", &show_demo_window);			// Edit bools storing our window open/close state
		//ImGui::SameLine();		

		ImGui::Checkbox("AYUDA", &AYUDA_activado);				//	maneja directo la ventana de ayuda con el true/false
		ImGui::SameLine(); 
		if (ImGui::Button("Reset Records"))	{
			limpiar_records();
		}
		
		/*
			ImGui::BeginTooltip();
			ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
			ImGui::TextUnformatted("CLICK");
			ImGui::PopTextWrapPos();
			ImGui::EndTooltip();
				*/
		

		//	actividades principales del programa
		ImGui::RadioButton("Salto Vertical",		&ACTIVIDADelegida, 0); 
		ImGui::SameLine();
		ImGui::RadioButton("Salto en Largo",		&ACTIVIDADelegida, 1); 
		ImGui::SameLine();
		ImGui::RadioButton("Selección",				&ACTIVIDADelegida, 2); 
		ImGui::SameLine();
		ImGui::RadioButton("Posturas",				&ACTIVIDADelegida, 3);


		switch (ACTIVIDADelegida) {
		case SALTO_VERTICAL:	//	0
			ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Salto Vertical");		//  yellow
			if (ACTIVIDADprevia != SALTO_VERTICAL) {	//	si viene de otra actividad...
				ACTIVIDADprevia = SALTO_VERTICAL;		//	actualizo previa
				limpiar_records();						//	limpio records
				SALTOstatus		= 0;					//	salto listo para empezar
				CONTANDO		= false;
			}
			break;
		case SALTO_LARGO:		//	1
			ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Salto en largo");		//  yellow
			if (ACTIVIDADprevia != SALTO_LARGO) {		//	si viene de otra actividad...
				ACTIVIDADprevia	= SALTO_LARGO;			//	actualizo previa
				limpiar_records();						//	limpio records
				SALTOstatus		= 0;					//	salto listo para empezar
				CONTANDO		= false;
			}
			break;
		case JUEGOS:			//	2, puede estar activo durante los saltos...
			//			todavía no tenemos actividades de juegos...
			ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Selección");			//  yellow
			if (ACTIVIDADprevia != JUEGOS) {											//	si viene de otra actividad...
				ACTIVIDADprevia = JUEGOS;												//	actualizo previa
				limpiar_records();														//	limpio records
				SALTOstatus		= 0;													//	salto listo para empezar
				CONTANDO		= false;
			}
			break;

		case POSTURAS:			//	3, para identificar posturas fijas
			//			
			ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Posturas");				//  yellow
			if (ACTIVIDADprevia != POSTURAS) {											//	si viene de otra actividad...
				ACTIVIDADprevia = POSTURAS;												//	actualizo previa
				limpiar_records();														//	limpio records
				SALTOstatus		= 0;													//	salto listo para empezar
				CONTANDO		= false;
			}
			break;

		case FLEXIONES:			//	4, contador de repeticiones de flexiones de brazo
			//			
			ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Posturas");				//  yellow
			if (ACTIVIDADprevia != FLEXIONES) {											//	si viene de otra actividad...
				ACTIVIDADprevia = FLEXIONES;											//	actualizo previa
				limpiar_records();														//	limpio records
				SALTOstatus		= 0;													//	salto listo para empezar
				CONTANDO		= false;
			}
			break;

		}


		if (!ImGui::CollapsingHeader("Usuario"))
		{
			ImGui::Text("%s %s", usuario_nombre, usuario_dni);
			ImGui::Separator();

			//  el alto del selector
			//ImGui::PushStyleVar(ImGuiStyleVar_GrabMinSize, 10);
			//  el color de hover
			//ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, (ImVec4)ImColor::HSV(1.0f, 0.6f, 0.5f));
			//  
			//ImGui::PushStyleColor(ImGuiCol_FrameBg, (ImVec4)ImColor::HSV(2 / 7.0f, 0.5f, 0.5f));
			//  activo
			//ImGui::PushStyleColor(ImGuiCol_FrameBgActive, (ImVec4)ImColor::HSV(3 / 7.0f, 0.7f, 0.5f));
			//  grab
			//ImGui::PushStyleColor(ImGuiCol_SliderGrab, (ImVec4)ImColor::HSV(4 / 7.0f, 0.9f, 0.9f));
			//ImGui::PushStyleColor(ImGuiCol_Text, (ImVec4)ImColor::HSV(4 / 7.0f, 0.9f, 0.9f));
			//ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
			//  el slider vertical
			//ImGui::VSliderFloat("##altura", ImVec2(80, 200), &altura, 0.0f, 2.5f, "Altura\n%03.2fm");
			//ImGui::PopStyleVar();
			//ImGui::PopStyleColor(1);
			//  altura

			//ImGui::PushFont(font24);
			if (!altura_validada) {
				//ImGui::SliderFloat("##altura", &altura, 0.0f, 2.5f, "Altura %03.2fm");
				ImGui::SliderInt("##altura", &alturaMM, 50, 300, "Altura %3dcm");
				//ImGui::SliderInt("##altura2", &envergaduraMM, 50, 300, "Envergadura %3dcm");
				alturaOK = alturaMM;
			}
			//ImGui::PopFont();
			ImGui::SameLine();
			ImGui::Checkbox("##altura_check", &altura_validada);                                       //  valida distancia
			ImGui::SameLine(); HelpMarker("Click para verificar la altura");
			//  alcance
			if (altura_validada) {
				ImGui::SliderInt("##alcance", &alturaMM, 50, 300, "Alcance %3dmm");
				ImGui::SameLine();
				ImGui::Checkbox("##alcance_check", &alcance_validado);                                  //  valida alcance
				ImGui::SameLine(); HelpMarker("Click para validar el alcance");
				alcanceOK = alturaMM;
			}
			
			if (!envergadura_valid) {
				ImGui::SliderInt("##altura2", &envergaduraMM, 50, 300, "Envergadura %3dcm");
				envergaduraOK = envergaduraMM;
			}
			//ImGui::PopFont();
			ImGui::SameLine();
			ImGui::Checkbox("##envergadura_check", &envergadura_valid);                                 //  valida envergadura
			ImGui::SameLine(); HelpMarker("Click para verificar la envergadura");

		}

		if (!ImGui::CollapsingHeader("Salto Actual & Mediciones"))
		{
			ImGui::Text("Estado: %u",SALTOstatus);
			ImGui::SliderInt("##distancia", &distancia, 25, 400, "Distancia %dcm");						//  muestra distancia 
			ImGui::PushFont(font24);																	//  cambio a 24
			
			switch (ACTIVIDADelegida) {
			case SALTO_VERTICAL:	//	0
				ImGui::Text("Altura obtenida: "); ImGui::SameLine();
				break;
			case SALTO_LARGO:		//	1
				ImGui::Text("Largo obtenido: "); ImGui::SameLine();
				break;
			case JUEGOS:			//	2
			case POSTURAS:			//	3
			case FLEXIONES:			//	4

				break;
			}						

			//ImGui::Text("Altura obtenida: "); ImGui::SameLine();
			ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), " %03.0fmm", RedonD(SALTO));				//	altura
			ImGui::Text("Tiempo de vuelo:  "); ImGui::SameLine();
			ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "%05.3fs", /*RedonD(1.0 * SALTOtiempo)*/ SALTOtiempo);	//	tiempo (ver si es eso o el doble)
			ImGui::PopFont();																			//  vuelvo a 20
			ImGui::Separator();
			//ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 1.0f), "Altura maxima  : %06.2fcm", RedonD(0.1 * SALTOmax));	//	saltomax
			//ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 1.0f), "Altura minima  : %06.2fcm", RedonD(0.1 * SALTOmin));	//	saltomin
			//ImGui::Separator();
			//int func = { 1 2 3 4 5 4 3 2 6 7 8 };
			if (SALTOfin > 0) {
				//							podríamos ajustar los valores máximos y mínimos del salto para que queden mejor
				//							si vengo de un salto exitoso, la cantidad de datos es SALTOfin >0 sinó es -1
				ImGui::PlotLines("##Lines", SALTOperfil, SALTOfin, 0, NULL, 0.0f, 500.0f, ImVec2(0, 80));				
			}
			else {
				ImGui::PlotLines("##Lines", SALTOperfil, IM_ARRAYSIZE(SALTOperfil), 0, NULL, -100.0f, 500.0f, ImVec2(0, 80));
			}
			
			//ImGui::PlotHistogram("Histogram", func, NULL, display_count, 0, NULL, -1.0f, 1.0f, ImVec2(0, 80));


		}

		//	
		if (!ImGui::CollapsingHeader("Mejor Salto"))
		{
			ImGui::PushFont(font40);																	//  cambio a 40
			ImGui::Text("Record");																		//	RECORD
			ImGui::Separator();	

			switch (ACTIVIDADelegida) {
			case SALTO_VERTICAL:	//	0
				ImGui::Text("Altura: "); ImGui::SameLine();
				break;
			case SALTO_LARGO:		//	1
				ImGui::Text("Largo: "); ImGui::SameLine();
				break;
			case JUEGOS:			//	2
			case POSTURAS:			//	3
			case FLEXIONES:			//	4

				break;
			}

			//ImGui::Text("Altura: "); ImGui::SameLine();
			ImGui::TextColored(ImVec4(0.9f, 0.5f, 0.0f, 1.0f), " %03umm", (SALTOrecord));		//	altura maxima
			ImGui::Text("Tiempo: "); ImGui::SameLine();
			ImGui::TextColored(ImVec4(0.9f, 0.5f, 0.0f, 1.0f), "%05.3fs", (TIEMPOrecord));		//	tiempo maximo era RedonD(.)
			ImGui::PopFont();			//  vuelvo a 20

			//  meter un plot acá de la evolución del salto
			//static bool animate = false;            //  variable de animación
			//ImGui::Checkbox("Animate", &animate);   //  check de animacion

			//float t[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
			//float arr[] = { 0.6f, 0.1f, 1.0f, 0.5f, 0.92f, 0.1f, 0.2f };
			//float arr[] = { 0.0f, 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f };
			//for (int kk = 0; kk < (IM_ARRAYSIZE(arr)); kk++) {
			//	arr[kk] = t[kk] * t[kk];
			//}
			//ImGui::PlotLines("##salto actual", SALTOperfil, IM_ARRAYSIZE(arr));

			/*
			// Create a dummy array of contiguous float values to plot
			// Tip: If your float aren't contiguous but part of a structure, you can pass a pointer to your first float and the sizeof() of your structure in the Stride parameter.
			static float values[90] = {};
			static int values_offset = 0;
			static double refresh_time = 0.0;
			if (!animate || refresh_time == 0.0)
				refresh_time = ImGui::GetTime();
			while (refresh_time < ImGui::GetTime()) // Create dummy data at fixed 60 hz rate for the demo
			{
				static float phase = 0.0f;
				values[values_offset] = cosf(phase);
				values_offset = (values_offset + 1) % IM_ARRAYSIZE(values);
				phase += 0.10f * values_offset;
				refresh_time += 1.0f / 60.0f;
			}
			ImGui::Separator();
			// Plots can display overlay texts
			// (in this example, we will display an average value)
			{
				float average = 0.0f;
				for (int n = 0; n < IM_ARRAYSIZE(values); n++)
					average += values[n];
				average /= (float)IM_ARRAYSIZE(values);
				char overlay[32];
				sprintf(overlay, "avg %f", average);
				ImGui::PlotLines("Lines", values, IM_ARRAYSIZE(values), values_offset, overlay, -1.0f, 1.0f, ImVec2(0, 80));
			}
			*/
			ImGui::Separator();			
		}

		//
		if (!ImGui::CollapsingHeader("Registros previos"))
		{
			ImGui::PushFont(font28);
			ImGui::Text("Cantidad de Saltos"); ImGui::SameLine();
			ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 1.0f), "%03u", repeticiones);								//	repeticiones
			ImGui::PopFont();
			ImGui::Separator();
			if (repeticiones < 40) {		//		esto hay que arreglarlo... corta a los 40 saltos

				switch (ACTIVIDADelegida) {
				case SALTO_VERTICAL:	//	0
					ImGui::Text("Altura del Salto: "); 
					break;
				case SALTO_LARGO:		//	1
					ImGui::Text("Largo del Salto: "); 
					break;
				case JUEGOS:			//	2
				case POSTURAS:			//	3
				case FLEXIONES:			//	4

					break;
				}

				//ImGui::Text("Altura del Salto");
				//ImGui::PlotHistogram("##HSaltos", saltos, IM_ARRAYSIZE(saltos), 0, NULL, 0.0, 1000.0, ImVec2(0, 80));
				ImGui::PlotHistogram("##HSaltos",	saltos,		IM_ARRAYSIZE(saltos),  0, NULL, 0.0, (float)(SALTOrecord), ImVec2(0, 80));	
				ImGui::Text("Tiempo de vuelo del salto");
				ImGui::PlotHistogram("##HTiempos",	tiempos,	IM_ARRAYSIZE(tiempos), 0, NULL, 0.0, (float)(TIEMPOrecord), ImVec2(0, 80));
			}
			
		}
		ImGui::End();
		// **********************************************************************************


		// **********************************************************************************
		//  ventana de la camara
		//  posición
		ImGui::SetNextWindowPos(ImVec2(5, 5), ImGuiCond_Always);
		//  tamaño
		ImGui::SetNextWindowSize(ImVec2(1280, 800), ImGuiCond_Always);		
        // 
		ImGui::Begin("Camara 3D");
		if (skeleton_mat.empty()) {
			ImGui::Text("CONECTE CAMARA REALSENSE");
		}
		else 
		{
			glTexImage2D(GL_TEXTURE_2D, 0, 3, skeleton_mat.cols, skeleton_mat.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, skeleton_mat.data);
			ImGui::Image((void*)(intptr_t)textureTrash, ImVec2(skeleton_mat.cols, skeleton_mat.rows));
		}		
		ImGui::End();
		// **********************************************************************************


		// **********************************************************************************
		//  ventana de la AYUDA
		//	se maneja directo desde un checkbox 
		if (AYUDA_activado) {			
			ImGui::SetNextWindowPos(ImVec2(10, 35), ImGuiCond_Always);		//  posición			
			ImGui::SetNextWindowSize(ImVec2(550, 80), ImGuiCond_Always);	//  tamaño
			// 
			ImGui::Begin("Ayuda");			
			ImGui::PushFont(font40);
			switch (ACTIVIDADelegida) {
			case SALTO_VERTICAL:	//	para el salto vertical
				switch (SALTOstatus) {
				case 0:				//	está torcido
					ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 1.0f), "TORCIDO");
					break;
				case 1:				//	inestable
					ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 1.0f), "POSE CORRECTA. QUIETO");
					break;
				case 2:				//	listo para sentadilla
					ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 1.0f), "SENTADILLA A 90º");
					break;
				case 3:				//	listo para salto
					ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 1.0f), "SALTE!");
					break;
				case 4:				//	saltando
					ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 1.0f), "SALTANDO!");
					break;
				}
				break;
			case SALTO_LARGO:		//	para el salto en largo
				switch (SALTOstatus) {
				case 0:				//	detectar puntas del pié adelante
					ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 1.0f), "PUNTAS DE PIE");
					break;
				case 1:				//	??
					ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 1.0f), "");
					break;
				case 2:				//	listo para sentadilla
					ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 1.0f), "SENTADILLA A 90º");
					break;
				case 3:				//	listo para salto
					ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 1.0f), "SALTE!");
					break;
				case 4:				//	saltando
					ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 1.0f), "SALTANDO!");
					break;
				}
				break;
			case JUEGOS:			//	para los juegos interactivos
				ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 1.0f), "toque los circulos");
				break;
			}
			ImGui::PopFont();
			ImGui::End();
		}
		// **********************************************************************************

		// **********************************************************************************
		// 3. Ventana de estado
		if (show_status_window)
		{
			//  posición
			ImGui::SetNextWindowPos(ImVec2(5, 810), ImGuiCond_Always);
			//  tamaño
			ImGui::SetNextWindowSize(ImVec2(1280, 170), ImGuiCond_Always);

			ImGui::Begin("Estado", &show_status_window);   //

			ImGui::SetNextItemWidth(500);
			ImGui::InputTextWithHint("##nombre", "Nombre", usuario_nombre, IM_ARRAYSIZE(usuario_nombre));
			ImGui::SameLine(); ImGui::SetNextItemWidth(150);
			ImGui::InputTextWithHint("##dni", "DNI", usuario_dni, IM_ARRAYSIZE(usuario_dni));
			ImGui::SameLine(); ImGui::SetNextItemWidth(75);
			ImGui::InputTextWithHint("##peso", "Peso", usuario_peso, IM_ARRAYSIZE(usuario_peso));
				
			if (altura_validada) {
				ImGui::Text("Altura: %03dmm", alturaOK);
			}

			if (alcance_validado) {
				ImGui::SameLine();
				ImGui::Text("   Alcance: %03dmm", alcanceOK);
			}

			if (envergadura_valid) {
				ImGui::SameLine();
				ImGui::Text("   Envergadura: %03dmm", envergaduraOK);
			}

			//ImGui::TextColored(ImVec4(1.0f, 0.0f, 1.0f, 1.0f), "Pink");     //pink
			//ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Salto en Alto");   //yellow

			ImGui::Separator();
			ImGui::SmallButton("Grabar datos");

			//if (ImGui::SmallButton("Cerrame"))
			//   show_status_window = false;
			ImGui::End();
			}
		// **********************************************************************************

		// **********************************************************************************
		//	VENTANA INTERACTIVA PARA SELECCIÓN DE ACTIVIDADES
		//			la ventana interactiva es una opción para manejar los menúes desde la pantalla. 
		//			este código se complementa con la opción CIRCULOS_activada 
		//			aparece un círculo verde que al tocarlo con la mano izquierda un tiempito, despliega un menú y botón rojo 
		//			el menú dá las opciones del programa, mientras que se selecciona al subir o bajar la mano,
		//			el click lo dá mantener la mano derecha sobre el botón rojo
		//			es un inicio
		if (VENTANAinteractiva) {					
			if (!(skeleton_mat.empty())) {
				ImGui::Begin("##Interactivo");
				
				//ImGui::Text("Usuario no detectado");
				//ImGui::SetNextWindowPos(ImVec2(articulacion[9].x, articulacion[9].y), ImGuiCond_Always);		//  posición		

				if (MANOinicio == true) {																		//	primera vez que detecta la mano?
					MANOinicio	= false;																		//	deja de ser la primera vez
					MANOinicial = ImVec2(RedonI(articulacion[9].x * 1.5), RedonI(articulacion[9].y * 1.66));	//	guardo la posición 
				}

				MANOactual = ImVec2(RedonI(articulacion[9].x * 1.5), RedonI(articulacion[9].y * 1.66));			//	mano actual							
				ImGui::SetWindowPos(MANOactual);																//	la ventana sobre la mano

				//ImGui::SetNextWindowPos(ImVec2(RedonI(articulacion[9].x * 1.5), RedonI(articulacion[9].y * 1.66)));

				ImGui::SetWindowSize(ImVec2(220, 220));															//  tamaño de la ventana
				
				centro2 = cv::Point(RedonI(MANOactual.x / 1.5) + 250, RedonI(MANOactual.y / 1.66));				//	circulito rojo acompañando a la misma altura

				//	sobre la mano izquierda aparece un menú con las siguientes opciones
				//ImGui::Text("MENUmano --> %d",MENUmano);
				ImGui::RadioButton("Salto Vertical",	&MENUmano, 0);
				ImGui::RadioButton("Salto en Largo",	&MENUmano, 1);
				ImGui::RadioButton("Selección",			&MENUmano, 2);
				ImGui::RadioButton("Posturas",			&MENUmano, 6);
				ImGui::RadioButton("Flexiones",			&MENUmano, 7);
				ImGui::RadioButton("Reset Rectords",	&MENUmano, 3);
				ImGui::RadioButton("Ver/Quitar Ayuda",	&MENUmano, 4);
				ImGui::RadioButton("Salir",				&MENUmano, 5);

				//	las cuales se van modificando automáticamente al subir o bajar la misma
				if ((MANOinicial.y - MANOactual.y) < -18) {
					MANOinicial = MANOactual;
					MENUmano++;
					if (MENUmano > 5) {
						MENUmano = 0;
					}
				}
				else {

					if ((MANOinicial.y - MANOactual.y) > 18) {
						MANOinicial = MANOactual;
						MENUmano--;
						if (MENUmano < 0) {
							MENUmano = 5;
						}
					}
				}
				//	la rutina de validación de una opción se hace aparte, por ahora en JUEGOS
				ImGui::End();
			}			
		}
		// **********************************************************************************


		// Rendering inevitable de la GUI
		ImGui::Render();
		int display_w, display_h;
		glfwGetFramebufferSize(window, &display_w, &display_h);
		glViewport(0, 0, display_w, display_h);
		glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);
	}

	// Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;   	  
}
// =========================================================================================================
// =========================================================================================================


// Initialize
void NuiTrack::initialize( const std::string& config_json )
{
    cv::setUseOptimized( true );				// no se que hace

    // Initialize NuiTrack
    tdv::nuitrack::Nuitrack::init( config_json );

    // Initialize Sensor
    initializeSensor();

    // Initalize Color Table for Visualization
    colors[0] = cv::Vec3b( 255,   0,   0 );		// TODOS LOS USUARIOS EN BLUE
	colors[1] = colors[0];
	colors[2] = colors[0];
	colors[3] = colors[0];
	colors[4] = colors[0];
	colors[5] = colors[0];
	colors[6] = colors[0];
	colors[7] = colors[0];
	/*
    colors[1] = cv::Vec3b(   0, 255,   0 );		// Green
    colors[2] = cv::Vec3b(   0,   0, 255 );		// Red
    colors[3] = cv::Vec3b( 255, 255,   0 );		// Cyan
    colors[4] = cv::Vec3b( 255,   0, 255 );		// Magenta
    colors[5] = cv::Vec3b(   0, 255, 255 );		// Yellow
	colors[6] = cv::Vec3b( 255, 255, 255);		// white
	colors[7] = cv::Vec3b(   0,   0,   0);		// black
	*/

	}

// Initialize Sensor
inline void NuiTrack::initializeSensor()
{

    // Set Device Config
	// always set mirror
	tdv::nuitrack::Nuitrack::setConfigValue("DepthProvider.Mirror", "true");		//always set mirror
	tdv::nuitrack::Nuitrack::setConfigValue("Skeletonization.ActiveUsers", "1");	// Only track primary user     agregada

	// Para activar el NUITRACK AI
	//set the "Skeletonization.Type" parameter to "CNN_HPE"
	//set the "DepthProvider.Depth2ColorRegistration" parameter to true
	// tdv::nuitrack::Nuitrack::setConfigValue("Skeletonization.Type", "CNN_HPE");	// 
	// tdv::nuitrack::Nuitrack::setConfigValue("DepthProvider.Depth2ColorRegistration", "true");	// 

    tdv::nuitrack::Nuitrack::setConfigValue("Realsense2Module.Depth.RawWidth", std::to_string( depth_width ) );			//
	tdv::nuitrack::Nuitrack::setConfigValue("Realsense2Module.Depth.ProcessWidth", std::to_string(depth_width));
	tdv::nuitrack::Nuitrack::setConfigValue("Realsense2Module.Depth.RawHeight", std::to_string(depth_height));			//
	tdv::nuitrack::Nuitrack::setConfigValue("Realsense2Module.Depth.ProcessHeight", std::to_string( depth_height ) );
	tdv::nuitrack::Nuitrack::setConfigValue("Realsense2Module.Depth.FPS", "60");							//	
	//tdv::nuitrack::Nuitrack::setConfigValue("Realsense2Module.Depth.FPS", std::to_string(FPSrealsense));							//	
	

	// Realsense RGB Module - force to 848x480 @ 60 FPS		
	tdv::nuitrack::Nuitrack::setConfigValue("Realsense2Module.RGB.RawWidth", std::to_string(color_width));
	tdv::nuitrack::Nuitrack::setConfigValue("Realsense2Module.RGB.RawHeight", std::to_string(color_height));
	tdv::nuitrack::Nuitrack::setConfigValue("Realsense2Module.RGB.ProcessWidth", std::to_string(color_width));
	tdv::nuitrack::Nuitrack::setConfigValue("Realsense2Module.RGB.ProcessHeight", std::to_string(color_height));
	//tdv::nuitrack::Nuitrack::setConfigValue("Realsense2Module.RGB.FPS", std::to_string(FPSrealsense));
	tdv::nuitrack::Nuitrack::setConfigValue("Realsense2Module.Depth.ProcessMaxDepth", "3800");
	tdv::nuitrack::Nuitrack::setConfigValue("Realsense2Module.RGB.FPS", "60");							//
	/*
	if (FPSrealsense > 10) {
		tdv::nuitrack::Nuitrack::setConfigValue("Realsense2Module.RGB.FPS", "30");							//
	}
	else {
		tdv::nuitrack::Nuitrack::setConfigValue("Realsense2Module.RGB.FPS", std::to_string(FPSrealsense));							//
	}
	*/
	real_Dfps = std::stoi( tdv::nuitrack::Nuitrack::getConfigValue("Realsense2Module.Depth.ProcessWidth"));
	//real_Dfps = std::stoi(tdv::nuitrack::Nuitrack::getConfigValue("Realsense2Module.Depth.FPS"));
	//real_Rfps = tdv::nuitrack::Nuitrack::getConfigValue("Realsense2Module.RGB.FPS");
    // Get Device Config

	max_distance		= std::stoi( tdv::nuitrack::Nuitrack::getConfigValue( "Realsense2Module.Depth.ProcessMaxDepth" ) );

    // Create Sensor
    depth_sensor		= tdv::nuitrack::DepthSensor::create();
	color_sensor		= tdv::nuitrack::ColorSensor::create();		//agregado de skeleton

    // Create Tracker
    user_tracker		= tdv::nuitrack::UserTracker::create();
	skeleton_tracker	= tdv::nuitrack::SkeletonTracker::create(); //agregado de skeleton

	T0 = 1 / FPSrealsense;		// período de muestreo
}

// Finalize
void NuiTrack::finalize()
{
    // Close Windows
    cv::destroyAllWindows();

    // Release NuiTrack
    tdv::nuitrack::Nuitrack::release();
}

// Update Data
void NuiTrack::update()
{
    // Update Frame
    updateFrame();

    // Update Depth
    updateDepth();

    // Update User
    updateUser();

	// ini de skeleton **********************
	// Update Color
	updateColor();

	// Update Skeleton
	updateSkeleton();
	// fin de skeleton **********************
}

// Update Frame
inline void NuiTrack::updateFrame()
{
    // Update Frame
    tdv::nuitrack::Nuitrack::update();
}

// Update Depth
inline void NuiTrack::updateDepth()
{
    // Retrieve Depth Frame
    depth_frame		= depth_sensor->getDepthFrame();	//get the depth frame
    // Retrive Frame Size
    depth_width		= depth_frame->getCols();
    depth_height	= depth_frame->getRows();
}

// Update User
inline void NuiTrack::updateUser()
{
    // Update Tracker
    try{
        tdv::nuitrack::Nuitrack::waitUpdate( user_tracker );
    }
    catch( const tdv::nuitrack::LicenseNotAcquiredException& ex ){
        throw std::runtime_error( "failed license not acquired" );
    }
    // Retrieve User Frame
    user_frame = user_tracker->getUserFrame();	// returns smart pointer to the last available USERFRAME
}

// ini de skeleton *******************************
// Update Color
inline void NuiTrack::updateColor()
{
	// Retrieve Color Frame
	color_frame		= color_sensor->getColorFrame();
	// Retrive Frame Size
	color_width		= color_frame->getCols();
	color_height	= color_frame->getRows();
}

// Update Skeleton
inline void NuiTrack::updateSkeleton()
{
	// Update Tracker
	try {
		tdv::nuitrack::Nuitrack::waitUpdate(skeleton_tracker);
	}
	catch (const tdv::nuitrack::LicenseNotAcquiredException& ex) {
		throw std::runtime_error("failed license not acquired");
	}
	// Retrieve Skeleton Data
	skeleton_data = skeleton_tracker->getSkeletons();	// returns smart pointer to the last available SkeletonData
}
// fin de skeleton ***************************


// Draw Data
void NuiTrack::draw()
{
    // Draw Depth
    drawDepth();
    // Draw User
    drawUser();
	//********************************
	// Draw Color
	drawColor();	// estaba comentado
	// Draw Skeleton
	drawSkeleton();
	//********************************
}

// Draw Depth
inline void NuiTrack::drawDepth()
{
    // Create cv::Mat form Depth Data
    const uint16_t* depth_data = depth_frame->getData();				//  returns the frame data 
    depth_mat = cv::Mat::zeros( depth_height, depth_width, CV_16UC1 );	//  completo con ceros el mat, 480 x 848 x uint16 (1 canal)

    #pragma omp parallel for
    for( int32_t index = 0; index < depth_mat.total(); index++ ){		// .total() returns the total number of array elements
        const uint16_t depth = depth_data[index];						// depth data tiene esa info 
        depth_mat.at<ushort>( index ) = depth;							// la asigno al mat de profundidad
		// Mat::at returns a reference to the specified array element
    }
}

// Draw User
inline void NuiTrack::drawUser()
{
    if( depth_mat.empty() ){
        return;
    }
	
    // Copy Depth Mat
    cv::cvtColor( depth_mat, user_mat, cv::COLOR_GRAY2BGR );						//convertir de gray a bgr
		//	cvtColor(input array, output array, color code, channels)  converts an image from one color space to another
		//	
	user_mat.convertTo( user_mat, CV_8U, -255.0 / max_distance, 255.0 );			// 0-max_distance -> 255(white)-0(black)
		//	converts an array to another data type with optional scaling
		//  (output array, type, optional scale factor, optial delta added to the scaled values

      //user_mat.convertTo( user_mat, CV_8U, 255.0 / max_distance , 0.0 ); // 0-max_distance -> 0(black)-255(white)
	//  convertTo --> converts an array to another data type with optional scaling 
    // Draw User Area
    const uint16_t* labels = user_frame->getData();						//
    #pragma omp parallel for
    for( int32_t y = 0; y < depth_height; y++ ){
        for( int32_t x = 0; x < depth_width; x++ ){
            const uint32_t index = y * depth_width + x;
            const uint16_t label = labels[index];
            if( label ){
				//			este muestra el rgb del usuario
				//user_mat.at<cv::Vec3b>(y, x) = color_mat.at<cv::Vec3b>(y,x);
				//			este muestra un color fijo
                user_mat.at<cv::Vec3b>( y, x ) = colors[label - 1];
            }
        }
    }

	/*
    // Draw Bounding Box [del usuario]	
    const std::vector<tdv::nuitrack::User> users = user_frame->getUsers();
    for( const tdv::nuitrack::User& user : users ){
		// para cada usuario
        const int32_t id = user.id;		
        const cv::Point point1 = { static_cast<int32_t>( user.box.left * depth_width ), static_cast<int32_t>( user.box.top * depth_height ) };
        const cv::Point point2 = { static_cast<int32_t>( user.box.right * depth_width ), static_cast<int32_t>( user.box.bottom * depth_height ) };
        //cv::rectangle( user_mat, point1, point2, colors[id - 1] );
		// además le quiero poner cuánto mide en mm
		const cv::Point point3 = { static_cast<int32_t>(user.box.right * depth_width), static_cast<int32_t>(user.box.top * depth_height) };
		cv::arrowedLine(user_mat, point2, point3, _black, 2, cv::LINE_AA, 0, 0.050);
		altura[user.id]		= (point2.y - point3.y);	//altura en pixels del usuario
		distancia[user.id]	= user.real.z;	//altura en pixels del usuario
		//cv::putText(user_mat, std::to_string(altura[user.id]), point3 + cv::Point(20, 40), cv::FONT_HERSHEY_TRIPLEX, 0.75, _black, 1, cv::LINE_AA);
    }
	*/

	// Draw Bounding Box [del usuario]	
	const std::vector<tdv::nuitrack::User> users = user_frame->getUsers();
	for (const tdv::nuitrack::User& user : users) {
		// para cada usuario
		const int32_t id = user.id;

		const cv::Point point1 = { static_cast<int32_t>(RedonD(double(user.box.left) * double(depth_width))), static_cast<int32_t>(RedonD(user.box.bottom * depth_height)) };
		//const cv::Point point1 = { static_cast<int32_t>(user.box.left * depth_width), static_cast<int32_t>(user.box.top * depth_height) };
		const cv::Point point2 = { static_cast<int32_t>(RedonD(double(user.box.right) * double(depth_width))), static_cast<int32_t>(RedonD(user.box.bottom * depth_height)) };		
		const cv::Point point3 = { static_cast<int32_t>(RedonD(double(user.box.right) * double(depth_width))), static_cast<int32_t>(RedonD(user.box.top * depth_height)) };
		const cv::Point point4 = { static_cast<int32_t>(RedonD(double(user.box.left) * double(depth_width))), static_cast<int32_t>(RedonD(user.box.top * depth_height)) };

		//	variables para salto en largo
		userTopLeft			= point4;			//
		userTopRight		= point3;			//
		userBottomLeft		= point1;			//
		userBottomRight		= point2;			//

		userCMx_MM			= user.real.x;		//
		userCMy_MM			= user.real.y;		//
		userCMz_MM			= user.real.z;		//

		//cv::rectangle( user_mat, point1, point2, colors[id - 1] );

		//	agrego una flecha para ver cuanto mide
		//cv::arrowedLine(user_mat, point2, point3, _black, 2, cv::LINE_AA, 0, 0.050);
		//	y otra para la envergadura
		//cv::arrowedLine(user_mat, point1, point2, _black, 2, cv::LINE_AA, 0, 0.050);

		//	ALTURA

		//altura[user.id] = (point2.y - point3.y);		// altura en pixels del usuario
		//altura_aux		= RedonI(point2.y - point3.y);
		//altura[user.id] = RedonI(filtroAB(altura_aux * 1.0, altura_old  * 1.0, alturaP_old));
		//altura[user.id] = (int)(0.5 * (double(altura[user.id]) + double(altura_old)));
		//altura_old		= altura[user.id];

		altura_aux		= RedonI(point2.y - point3.y);											//	en pixels
		altura			= RedonI(filtroAB(altura_aux * 1.0, altura_old * 1.0, alturaP_old));	//	
		altura			= (int)(0.5 * (double(altura) + double(altura_old)));					//	
		altura_old		= altura;																//	

		envergadura_aux = RedonI(point3.x - point4.x);														//	en pixels
		envergadura		= RedonI(filtroAB(envergadura_aux * 1.0, envergadura_old * 1.0, envergadurP_old));	//	
		envergadura		= (int)(0.5 * (double(envergadura) + double(envergadura_old)));						//	
		envergadura_old = envergadura;																		//	

		
		//envergadura = RedonI(point3.x - point4.x);

		//	DISTANCIA

		//distancia[user.id] = user.real.z;				// distancia en mm al usuario
		//distancia[user.id]	= RedonI(filtroAB(dist_aux * 1.0, distancia_old  * 1.0, distanciaP_old) );		
		dist_aux		= RedonI( (int) (0.1 * user.real.z));										//en mm lo paso a cm
		distancia		= RedonI(  filtroAB(dist_aux * 1.0, distancia_old * 1.0, distanciaP_old) );	// la dejo en cm
		distancia_old	= dist_aux;
		
		//cv::putText(user_mat, std::to_string(altura[user.id]), point3 + cv::Point(20, 40), cv::FONT_HERSHEY_TRIPLEX, 0.75, _black, 1, cv::LINE_AA);
		
	}
}


// ini de skeleton *****************************

// Draw Color
inline void NuiTrack::drawColor()
{
	//no hace falta la imagen RGB
	// Create cv::Mat form Color Data
	const tdv::nuitrack::Color3* color_data = color_frame->getData();
	color_mat = cv::Mat::zeros(color_height, color_width, CV_8UC3);
#pragma omp parallel for
	for (int32_t index = 0; index < color_mat.total(); index++) {
		const tdv::nuitrack::Color3 color = color_data[index];
		color_mat.at<cv::Vec3b>(index) = cv::Vec3b(color.blue, color.green, color.red);
	}	
}


// Draw Skeleton
inline void NuiTrack::drawSkeleton()
{
	
	//	voy a copiar user_mat en skeleton_mat, pero cambiando de tamaño para poder meter el texto
	skeleton_mat = cv::Mat::zeros(final_height, final_width, CV_8UC3);	// es de 800 x 1280

	/*
	// esto es de la cuenta de flexiones de biceps
	if (PROCESANDO == false) {
		PROCESANDO      = true;
		FLEX_biceps_l	= 0;				//  inicio contador de flexiones bíceps derecho
		repeticiones    = 0;				//  repeticiones
	}
	*/
	
	// este bloque es el permite manejar el reloj.
	//   quiero mostrar el período de muestreo ahora
	if (t_INIT == true) {					//	inicio de la detección del esqueleto
		t_INIT = false;
		t_start			= std::clock();		//  inicio	tomo el valor de tiempo		
	}
	else {
		t_INIT = true;
		t_end			= std::clock();														//	final
		t_taken			= round(1000.0 * (double(t_end) - (double)(t_start)) / double(CLOCKS_PER_SEC));	//	tiempo por muestra en milisegundos		

		if (t_taken < t_takenm) {
			t_takenm = t_taken;
		}
		if (t_taken > t_takenM) {
			t_takenM = t_taken;
		}
		conta++;
		if (conta > 15) {
			conta = 0;
			t_takenm = 250;
			t_takenM = 0;
		}
	}
	//           fin bloque reloj

	// se dibuja sobre el de color, hay que dibujarlo sobre el que tiene el usuario
	//if (color_mat.empty()) {
	if (user_mat.empty()) {
		return;
	}

	
	// Copio el user sobre el esqueleto
	//color_mat.copyTo(skeleton_mat);
	//cv::resize(user_mat, user_mat, cv::Size(final_width, final_height ), 0, 0, 1);
	user_mat.copyTo(skeleton_mat);

	// Draw Skeleton
	const std::vector<tdv::nuitrack::Skeleton> skeletons = skeleton_data->getSkeletons();

	for (const tdv::nuitrack::Skeleton& skeleton : skeletons) {
		// para cada esqueleto
		cont = 0;
		const int32_t id = skeleton.id;
		const std::vector<tdv::nuitrack::Joint> joints = skeleton.joints;
		for (const tdv::nuitrack::Joint& joint : joints) {
			artFULL[cont] = joint;
			// para cada articulación
			if (joint.confidence < 0.5) {					//  era 0.2
				//si estoy por debajo del valor de confianza mínimo la ignoro
				articulacion[cont]	= cv::Point(0, 0);		//  JMT, la pongo en el vértice							
				//art_depth[cont]		= 0;
				cont++;										//  sigo con el contador
				continue;
			}
			//  obtuvo un joint correcto y lo ubica como POINT en la imagen de acuerdo al ancho de la pantalla
			const cv::Point point	= { static_cast<int32_t>(joint.proj.x * color_width) , static_cast<int32_t>(joint.proj.y * color_height) };
			articulacion[cont]		= point;				//	el punto en la pantalla
			artFULL[cont]			= joint;				//	la articulación completa
			//cont++;
			//  pero voy a evitar las manos
			switch (cont)
			{
				/*
			case 3:					
				distance = depth_mat.at<uint16_t>(articulacion[3]);		//distancia de la articulación [2] en la imagen de profundidad
				//  muestro la distancia
				cv::putText(skeleton_mat, cv::format("d1 = %u",distance), cv::Point(Poffset + 300, 60), cv::FONT_HERSHEY_TRIPLEX, 1, _cyan, 1, cv::LINE_AA);
				// alternativo
				distance = joint.proj.z;				//  distancia de la articulación 2
				//out = "d2 = " + std::to_string(distance);
				//cv::putText(skeleton_mat, out, cv::Point(Poffset + 310, 90), cv::FONT_HERSHEY_TRIPLEX, 1, _cyan, 1, cv::LINE_AA);
				cv::putText(skeleton_mat, cv::format("d2 = %u", distance), cv::Point(Poffset + 300, 85), cv::FONT_HERSHEY_TRIPLEX, 1, _cyan, 1, cv::LINE_AA);				
				cv::putText(skeleton_mat, cv::format("d3 = %u", distancia[skeleton.id]), cv::Point(Poffset + 300, 115), cv::FONT_HERSHEY_TRIPLEX, 1, _cyan, 1, cv::LINE_AA);
				break;
				*/
			case 9:		//	mano izquierda
				if (ACTIVIDADelegida == JUEGOS) {
					cv::circle(skeleton_mat, point, 4, _green, -1);
					break;
				}
			case 15:	//	mano derecha
				if (ACTIVIDADelegida == JUEGOS) {
					cv::circle(skeleton_mat, point, 4, _red, -1);
					break;
				}
				//  cv::circle(skeleton_mat, point, 12, _red, -1);
				//	continue;
			case 4:			//	waist cintura centro de masa
				//distance = RedonI(1.0 * artFULL[04].real.y);
				//cv::putText(skeleton_mat, cv::format("%04u", distance), articulacion[cont], cv::FONT_HERSHEY_TRIPLEX, .5, _black, .25, cv::LINE_AA);
			case 6:
			case 12:
			case 17:
			case 21:
				if (ACTIVIDADelegida == SALTO_VERTICAL) {
					cv::circle(skeleton_mat, point, 8, _white, -2);
				}			
				//distance = depth_mat.at<uint16_t>(articulacion[cont]);
				//distance = joint.proj.y;
				//distance = artFULL[cont].real.y;
				//cv::putText(skeleton_mat, cv::format("%04u", distance), articulacion[cont], cv::FONT_HERSHEY_TRIPLEX, .5, _black, .25, cv::LINE_AA);
				//cv::circle(skeleton_mat, point, 6, colors[0], -1);
				break;
			default:
				//cv::circle(skeleton_mat, point, 5, _white, -1);
				if (ACTIVIDADelegida == SALTO_VERTICAL) {
					cv::circle(skeleton_mat, point, 3, _black, -1);
				}
				break;
			}
			cont++;
			//cv::circle(skeleton_mat, point, 8, colors[2], -1);
			//cv::circle(skeleton_mat, point, 5, colors[6], -1);			
			//cv::putText(skeleton_mat, "Hay esqueleto!", cv::Point(20, 20), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(200, 200, 50), 1, cv::LINE_AA);
		}
		//cv::putText(skeleton_mat, std::to_string (cont)  , cv::Point(20, 20), cv::FONT_HERSHEY_TRIPLEX, 3, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
		
		
		//  *****************  PROCESAMIENTO ABAJO

		dist		= RedonD(sqrt(pow(artFULL[2].real.x - artFULL[4].real.x, 2) + pow(artFULL[2].real.y - artFULL[4].real.y, 2)));
		escala		= (dist / RedonD(sqrt(pow(articulacion[2].x - articulacion[4].x, 2) + pow(articulacion[2].y - articulacion[4].y, 2))));
		alturaMM	= RedonI(0.1 * altura * escala);
		//alturaMM_2 = RedonI(0.1*(1140 + escala * (240-userTopRight.y) / (distancia*10.0)));

		envergaduraMM = RedonI(0.1*escala*envergadura);
		userCMx_PX	= articulacion[4].x;												//
		userCMy_PX	= articulacion[4].y;												//

		// armo el esqueleto para el SALTO VERTICAL
		const int esqueleto_ancho = 1;
		if (ACTIVIDADelegida == SALTO_VERTICAL) {

			Hueso(skeleton_mat, articulacion, 1, 2, _red,		esqueleto_ancho);		//head-neck
			Hueso(skeleton_mat, articulacion, 2, 3, _yellow,	esqueleto_ancho + 1);	//neck-torso
			Hueso(skeleton_mat, articulacion, 3, 4, _red,		esqueleto_ancho);		//torso-waist
																						//
			Hueso(skeleton_mat, articulacion, 5, 6, _red,		esqueleto_ancho);		//left collar-shoulder
			Hueso(skeleton_mat, articulacion, 6, 7, _red,		esqueleto_ancho);		//left shoulder-elbow
			Hueso(skeleton_mat, articulacion, 7, 8, _red,		esqueleto_ancho);		//left elbow-wrist
			Hueso(skeleton_mat, articulacion, 8, 9, _red,		esqueleto_ancho);		//left wrist-hand
																						//
			Hueso(skeleton_mat, articulacion, 11, 12, _red,		esqueleto_ancho);		//right collar-shoulder
			Hueso(skeleton_mat, articulacion, 12, 13, _red,		esqueleto_ancho);		//right shoulder-elbow 
			Hueso(skeleton_mat, articulacion, 13, 14, _red,		esqueleto_ancho);		//right shoulder-elbow 
			Hueso(skeleton_mat, articulacion, 14, 15, _red,		esqueleto_ancho);		//right wrist-hand
																						//
			Hueso(skeleton_mat, articulacion,  4, 21, _red,		esqueleto_ancho);		//right waist-hip
			Hueso(skeleton_mat, articulacion, 21, 22, _red,		esqueleto_ancho);		//right hip-knee
			Hueso(skeleton_mat, articulacion, 22, 23, _red,		esqueleto_ancho);		//right knee-ancle
																						//
			Hueso(skeleton_mat, articulacion,  4, 17, _red,		esqueleto_ancho);		//left waist-hip
			Hueso(skeleton_mat, articulacion, 17, 18, _red,		esqueleto_ancho);		//left hip-knee
			Hueso(skeleton_mat, articulacion, 18, 19, _red,		esqueleto_ancho);		//left knee-ancle	

			Hueso(skeleton_mat, articulacion, 23, 24, _red,		esqueleto_ancho + 2);	//right knee-ancle
			Hueso(skeleton_mat, articulacion, 19, 20, _red,		esqueleto_ancho + 2);	//left knee-ancle	

			//  algunos arcos internos
			ang_l_codo = Arco(skeleton_mat, 6, 7, 8, _black, -1);		// codo izquierdo del usuario de fremte a la cámara		
			ang_r_codo = Arco(skeleton_mat, 12, 13, 14, _yellow, -1);		// codo derecho
			//ang_l_hombro	= Arco(skeleton_mat,  05, 06, 07, _yellow, -1);		// hombro izq
			//ang_r_hombro	= Arco(skeleton_mat,  11, 12, 13, _yellow, -1);		// hombro derecho

		}
		

		//	JUEGUITO CON LOS CÍRCULOS
		//	si la muñeca derecha lo toca... cambia de color y si la mantiene 2 segundos y la otra mano está por encima, lo puede mover		
		if (ACTIVIDADelegida == JUEGOS) {	
			//	inicialmente CICULOcont es -1, acá toma el valor inicial
			if (CIRCULOcont < 0) {																			//	después de un tiempo que estoy arriba
				// entonces tomo del torso a la derecha y abajo
				centro = cv::Point(articulacion[3].x - 20, articulacion[3].y - 5);
				//centro		= cv::Point(articulacion[4].x - 200, articulacion[4].y - 50);					//	centro --> amarillo, condición inicial
				//centro2		= cv::Point(articulacion[4].x - 150, articulacion[4].y + 50);					//	centro2 --> rojo, si lo tocás perdés
			}
			cv::circle(skeleton_mat, centro, 8, _green, -2);												//	círculo verde
			cv::circle(skeleton_mat, centro2, 8, _red, -2);													//	círculo rojo
			//	PARA MANO IZQUIERDA
			if ((abs(articulacion[9].x - centro.x) < 25) && (abs(articulacion[9].y - centro.y) < 25)) {		//	si me acerco con la mano izq...
				cv::circle(skeleton_mat, centro, 8, _yellow, -2);											//	el verde cambia a amarillo
				CIRCULOcont++;			
				if (CIRCULOcont > 45) {																		//	después de un tiempo que estoy arriba
					VENTANAinteractiva = true;
					/*
					if (articulacion[15].y < articulacion[9].y) {
						centro = cv::Point(articulacion[9].x, articulacion[9].y);							//	me lo llevo a la mano a otro lado
						cv::circle(skeleton_mat, centro, 8, _fucsia, -2);									//	círculo fucsia
						//VENTANAinteractiva = true;
					}	
					*/
				}
			}
			else
			{
				CIRCULOcont = 0;
				if ((abs(articulacion[9].x - centro2.x) < 25) && (abs(articulacion[9].y - centro2.y) < 25)) {//	PERO si me acerco con la mano izq al círculo rojo...
					cv::circle(skeleton_mat, centro2, 36, _black, -2);													//	círculo rojo
				}
			}
			//	PARA MANO DERECHA
			if ((abs(articulacion[15].x - centro2.x) < 25) && (abs(articulacion[15].y - centro2.y) < 25)) {		//	si me acerco con la mano izq...
				cv::circle(skeleton_mat, centro2, 8, _yellow, -2);											//	el verde cambia a amarillo
				//
				//	esto se usaba para cambiar el valor del selector
				//SALTOstatus++;
				//if (SALTOstatus > 4) SALTOstatus = 0;
				//
				CIRCULO2cont++;
				if (CIRCULO2cont > 50) {																		//	después de un tiempo que estoy arriba

					cv::circle(skeleton_mat, centro2, 8, _fucsia, -2);									//	círculo fucsia

					switch (MENUmano) {
					case 0:		//			vertical
						ACTIVIDADelegida	= SALTO_VERTICAL;						
						VENTANAinteractiva	= false;
						break;
					case 1:		//			largo
						ACTIVIDADelegida	= SALTO_VERTICAL;						
						VENTANAinteractiva	= false;
						break;
					case 2:		//			seguir jugando
						ACTIVIDADelegida	= JUEGOS;						
						VENTANAinteractiva	= false;
						break;
					case 3:		//			reset records
						limpiar_records();
						VENTANAinteractiva	= false;
						break;
					case 4:		//			ayuda
						if (AYUDA_activado) {
							AYUDA_activado = false;
						}
						else {
							AYUDA_activado = true;
						}
						VENTANAinteractiva = false;
						break;
					case 5:		//			salir
						VENTANAinteractiva = false;
						break;
					}
					
					/*
					if (articulacion[9].y < articulacion[15].y) {
						//centro2 = cv::Point(articulacion[15].x, articulacion[15].y);						//	me lo llevo a la mano a otro lado
						cv::circle(skeleton_mat, centro2, 8, _fucsia, -2);									//	círculo fucsia
											
						//
						//	con esta opción voy a cambiar el selector validado.
						//SALTOstatus = 1;
						//

					}
					*/
				}
			}
			else
			{
				CIRCULO2cont = 0;
				if ((abs(articulacion[15].x - centro.x) < 25) && (abs(articulacion[15].y - centro.y) < 25)) {//	PERO si me acerco con la mano izq al círculo rojo...
					cv::circle(skeleton_mat, centro, 36, _black, -2);													//	círculo rojo
				}
			}
		}



		//	CONTADOR DE FLEXIONES BICEPS DERECHO, de 30 a 145 cuando el ángulo del hombro esté por encima de los 175º
		//
		if (ACTIVIDADelegida == FLEXIONES) {
			if (abs(ang_l_hombro) < 150) {
				FLEX_biceps_l = 0;		//	hay que empezar de cero
				CONTANDO = false;
			}
			else
			{
				switch (FLEX_biceps_l) {
				case 0:						//	inicio, empieza a contar cuando está extendido
					if (ang_l_codo >= 145) {
						FLEX_biceps_l = 1;
						repeticiones = 0;
						//	enciendo el cronómetro
						if (CONTANDO == false) {
							CONTANDO = true;		//	contando a true
							t_cur = clock();		//	y reseteo
						}
					}
					break;
				case 1:						//	de extendido pasa a reducir el ángulo
					if (ang_l_codo < 145) {
						FLEX_biceps_l = 2;
					}
					break;
				case 2:						//	hasta que llegue al mínimo
					if (ang_l_codo < 30) {
						FLEX_biceps_l = 3;
					}
					break;
				case 3:						//	llegó al mínimo
					if (ang_l_codo > 30) {
						FLEX_biceps_l = 4;
					}
					break;
				case 4:
					if (ang_l_codo > 145) {
						FLEX_biceps_l = 1;
						repeticiones++;
					}
					break;
				}
				cv::putText(skeleton_mat, std::to_string(FLEX_biceps_l), (cv::Point(Poffset + 200, 15)), cv::FONT_HERSHEY_TRIPLEX, .75, _cyan, 1, cv::LINE_AA);
			}
		}

		
		// algunos cambios dependiendo de los ángulos
		// 1) si los codos son cero hago como un hueso más grande desde la muñeca al hombro
		//if (abs(ang_l_codo - 90.0) < 45.00) {
			//Hueso(skeleton_mat, articulacion, 6, 8, _white, 7);
			//ang_l_codo	= Arco(skeleton_mat, 6, 7, 8, _red, -1);
			//distance	= depth_mat.at<uint16_t>(articulacion[7]);
			//cv::putText(skeleton_mat, std::to_string(distance), (articulacion[7]+cv::Point(60, 60)), cv::FONT_HERSHEY_TRIPLEX, 0.6, _black, 1, cv::LINE_AA);			
		//}
		
		/*
		if (abs(articulacion[12].y - articulacion[14].y) < 4.00) {
			Hueso(skeleton_mat, articulacion, 12, 14, _white, 7);	//	hago com un hueso más grande en blanco
			// aprovecho y reseteo el maximo del salto
			SALTOrecord  = 0;
			SALTOmax     = 3000;
			repeticiones = 0;			
			//	enciendo el cronómetro
			if (CONTANDO == false) {
				CONTANDO	= true;		//	contando a true
				t_cur		= clock();	//	y reseteo
			}
			
		}
		*/

		/*if (abs(ang_r_hombro - ang_l_hombro) < 3.00) {
			Hueso(skeleton_mat, articulacion, 6, 12, _white, 10);	//de hombro a hombro
			if (CONTANDO == true) {
				CONTANDO = false;
			}
		}*/

		//	quiero mostrar la distancia entre las manos
		//	distancia entre articulaciones 10 y 16
		/*if ((articulacion[9].x > 0) && (articulacion[15].x > 0)){
			distance = sqrt(pow(artFULL[9].real.x - artFULL[15].real.x, 2) + pow(artFULL[9].real.y - artFULL[15].real.y, 2) + pow(artFULL[9].real.z - artFULL[15].real.z, 2));
			cv::arrowedLine(skeleton_mat, articulacion[9], articulacion[15], _cyan, 3, cv::LINE_AA, 0, 0.050);
			//cv::putText(skeleton_mat, std::to_string(distance), cv::Point(Poffset + 100, 360), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, _white, 1, cv::LINE_AA);
			const double escala = distance / (sqrt(pow(articulacion[9].x - articulacion[15].x, 2) + pow(articulacion[9].y - articulacion[15].y, 2)));
			//cv::putText(skeleton_mat, cv::format("Altura: %4.0f",altura[skeleton.id]*escala), cv::Point(Poffset + 100, 400), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, _white, 1, cv::LINE_AA);
		}*/

		//cv::putText(skeleton_mat, std::to_string(skeleton_mat.type()), cv::Point(200, 200), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, _red, 1, cv::LINE_AA);
		
		//cv::ellipse(skeleton_mat, articulacion[7], cv::Size(30, 30), ANGrot, ANGi, ANGf, _yellow, 2, 8);
		//cv::line(skeleton_mat, articulacion[3], articulacion[4], cv::Scalar(0, 0, 255), 3);

		//				RUTINA DE SALTO

		//Hueso(skeleton_mat, articulacion, 6, 12, _red, esqueleto_ancho + 3);	//	línea de hombros
		//cv::putText(skeleton_mat, cv::format("%4.0f", RedonD(artFULL[06].real.z)), cv::Point(articulacion[06].x - 20, articulacion[06].y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, _white, 1, cv::LINE_AA);
		//cv::putText(skeleton_mat, cv::format("%4.0f", RedonD(artFULL[12].real.z)), cv::Point(articulacion[12].x + 20, articulacion[12].y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, _white, 1, cv::LINE_AA);
	
		//Hueso(skeleton_mat, articulacion, 17, 21, _red, esqueleto_ancho + 3);	//	línea de caderas
		//cv::putText(skeleton_mat, cv::format("%4.0f", RedonD(artFULL[17].real.z)), cv::Point(articulacion[17].x - 20, articulacion[17].y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, _white, 1, cv::LINE_AA);
		//cv::putText(skeleton_mat, cv::format("%4.0f", RedonD(artFULL[21].real.z)), cv::Point(articulacion[21].x + 20, articulacion[21].y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, _white, 1, cv::LINE_AA);
	
		//Hueso(skeleton_mat, articulacion, 19, 23, _red, esqueleto_ancho + 3);	//	línea de pies
		//cv::putText(skeleton_mat, cv::format("%4.0f", RedonD(artFULL[19].real.z)), cv::Point(articulacion[19].x - 20, articulacion[19].y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, _white, 1, cv::LINE_AA);
		//cv::putText(skeleton_mat, cv::format("%4.0f", RedonD(artFULL[23].real.z)), cv::Point(articulacion[23].x + 20, articulacion[23].y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, _white, 1, cv::LINE_AA);

		//	algunas variables internas del salto
		int		DIFhombrosCM	=  00;	//	diferencia entre los hombros 6 y 12 en z en CM
		int		DIFcaderasCM	=  00;	//	diferencia entre los caderas 17 y 21 en z en CM
		int		DIFpiesCM		=  00;	//	diferencia entre los pies 19 y 23 en z en CM
		int		DISThombrosCM	=  00;	//	distancias promedio entre HOMBROS derecho e izquierdo
		int		DISTcaderasCM	=  00;	//	distancias promedio entre CADERAS derecho e izquierdo
		int		DISTpiesCM		=  00;	//	distancias promedio entre PIES derecho e izquierdo
		int		DIF_dist		=  00;	//	diferencia entre distancias hombro cadera

		int		DIFumbralCM		=  05;	//	umbral para estar cruzado derecha izquierda para hombro o cadera en CM		
		int		STABLEthr		=  60;	//	umbral de estabilidad para dar por hecho que una medida es estable en SAMPLES

		int		CMumbralMM		= 100;	//	es el umbral que le sumo y le resto a la base de cintura fija
		int		CMdistUmbMM		= 220;	//	umbral para la distancia

		//								muestro el estado del salto
		//cv::putText(skeleton_mat, std::to_string(SALTOstatus), (cv::Point(Poffset + 200, 15)), cv::FONT_HERSHEY_TRIPLEX, .5, _cyan, 1, cv::LINE_AA);
		

		//		diferencias entre articulaciones típicas del salto: hombros, caderas y pies en CM
		DIFhombrosCM	= RedonI(0.1 * abs(artFULL[12].real.z - artFULL[06].real.z));
		DIFcaderasCM	= RedonI(0.1 * abs(artFULL[21].real.z - artFULL[17].real.z));
		DIFpiesCM		= RedonI(0.1 * abs(artFULL[23].real.z - artFULL[19].real.z));
		//		distancias promedio (hombros caderas piés) en CM
		DISThombrosCM	= RedonI(0.5 * 0.1 * abs(artFULL[12].real.z + artFULL[06].real.z));
		DISTcaderasCM	= RedonI(0.5 * 0.1 * abs(artFULL[12].real.z + artFULL[06].real.z));
		DISTpiesCM		= RedonI(0.5 * 0.1 * abs(artFULL[12].real.z + artFULL[06].real.z));
		//		distancia hombros cadera en CM
		DIF_dist		= RedonI(1.0 * abs(DISThombrosCM-DISTcaderasCM));

		//ALTURAhombroMM	= RedonI(1.0 * 0.5 * ( RedonI(artFULL[06].real.y + artFULL[12].real.y)) );
		//ALTURAcaderaMM	= RedonI(1.0 * 0.5 * ( RedonI(artFULL[17].real.y + artFULL[21].real.y)) );
		//ALTURApiesMM	= RedonI(1.0 * 0.5 * ( RedonI(artFULL[19].real.y + artFULL[23].real.y)) );
		//ALTURAwaistMM	= RedonI(1.0 * artFULL[04].real.y);													//	altura en MM de la cintura

		//cv::putText(skeleton_mat, cv::format("%d[%d]@%d", DIFhombrosCM,ALTURAhombroMM, DISThombrosCM), cv::Point(articulacion[12].x + 20, articulacion[12].y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, _yellow, 1, cv::LINE_AA);
		//cv::putText(skeleton_mat, cv::format("%d[%d]@%d", DIFcaderasCM, ALTURAhombroMM, DISTcaderasCM), cv::Point(articulacion[21].x + 20, articulacion[21].y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, _yellow, 1, cv::LINE_AA);
		//cv::putText(skeleton_mat, cv::format("%d[%d]@%d", DIFpiesCM, ALTURApiesMM, DIST_p), cv::Point(articulacion[23].x + 20, articulacion[23].y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, _yellow, 1, cv::LINE_AA);

		//cv::putText(skeleton_mat, cv::format("%d@%d", DIFhombrosCM,  DISThombrosCM), cv::Point(articulacion[12].x + 20, articulacion[12].y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, _yellow, 1, cv::LINE_AA);
		//cv::putText(skeleton_mat, cv::format("%d@%d", DIFcaderasCM,  DISTcaderasCM), cv::Point(articulacion[21].x + 20, articulacion[21].y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, _yellow, 1, cv::LINE_AA);
		//cv::putText(skeleton_mat, cv::format("%d@%d", DIFpiesCM,  DIST_p), cv::Point(articulacion[23].x + 20, articulacion[23].y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, _yellow, 1, cv::LINE_AA);

		if (ACTIVIDADelegida == SALTO_VERTICAL) {		

			switch (SALTOstatus) {

				// #############################################################################################
				case 0:							//	TORCIDO! no hay distancia correcta ni estable 
					//								si entre hombros y caderas tengo una distancia menor que el umbral paso a 1
					//								sino, muestro que estoy torcido con color rojo
					SALTOcontadorS	= 0;		//	contador para Estabilidad de la medición
					CONTANDO		= false;	//	timer --> off
					SALTANDO		= false;	//	no está saltando
					//
					if (((DIFhombrosCM < DIFumbralCM) && (DIFcaderasCM < DIFumbralCM)))
					{						
						//						si estamos bien orientados en las dos, la linea de hombro y caderas se ve fluo						
						//						--> no estamos controlando que el tipo esté derecho...
						//
						Hueso(skeleton_mat, articulacion, 06, 12, _fluo, esqueleto_ancho + 3);	//	línea de hombros			
						Hueso(skeleton_mat, articulacion, 17, 21, _fluo, esqueleto_ancho + 3);	//	línea de caderas	

						SALTOstatus		= 1;	//	paso al estado 1 buscando estabilidad de la lectura
						SALTOcontadorS	= 0;	//	con el contador a cero
					}
					else 
					{
						//						pero si no estamos bien orientados, los huevos de hombro y cadera se ven rojos...
						//
						if (DIFhombrosCM < DIFumbralCM) {		//	hombros torcidos derecha/izquierda? 
							Hueso(skeleton_mat, articulacion, 6, 12, _fluo, esqueleto_ancho + 3);		//	línea de hombros correcta, fluo
						}
						else 
						{
							Hueso(skeleton_mat, articulacion, 6, 12, _red, esqueleto_ancho + 3);		//	línea de hombros torcida, roja
						}
						if (DIFcaderasCM < DIFumbralCM) {
							Hueso(skeleton_mat, articulacion, 17, 21, _fluo, esqueleto_ancho + 3);		//	línea de caderas correcta, fluo
						}
						else 
						{
							Hueso(skeleton_mat, articulacion, 17, 21, _red, esqueleto_ancho + 3);		//	línea de caderas
						}
						//																					LOS PIES NO SE TIENEN EN CUENTA ACÁ
						if (DIFpiesCM < (2 * DIFumbralCM) ) {
							Hueso(skeleton_mat, articulacion, 19, 23, _fluo, esqueleto_ancho + 3);		//	línea de pies en línea
						}
						else 
						{
							Hueso(skeleton_mat, articulacion, 19, 23, _red, esqueleto_ancho + 3);		//	algún pié adelantado
						}
					}
					break;
					// #############################################################################################
				case 1:		//			estamos bien orientados DERECHA IZQUIERDA, pero no es estable
					//					repito la misma condición del estado 0
					if ((DIFhombrosCM < DIFumbralCM) && (DIFcaderasCM < DIFumbralCM)) {
						//				la orientacion está dentro del umbral máximo
						//				muestro las lines de hombro y cadera en fluo
						Hueso(skeleton_mat, articulacion, 06, 12, _fluo, esqueleto_ancho + 3);	//	línea de hombros			
						Hueso(skeleton_mat, articulacion, 17, 21, _fluo, esqueleto_ancho + 3);	//	línea de caderas					
						//				y aumento el contador para buscar la estabilidad
						SALTOcontadorS++;		//
						if (SALTOcontadorS > STABLEthr)
						{
							//			después de X veces, paso de estado
							SALTOstatus		= 2;
							SALTOcontadorS	= 0;
							//			y acá dejo marcado los puntos para construir las lineas fijas que serán la base del salto
							SALTOhombro06	= cv::Point(articulacion[06].x - 20, articulacion[12].y);
							SALTOhombro12	= cv::Point(articulacion[12].x + 20, articulacion[12].y);		//	hombros
							SALTOwaist04a	= cv::Point(articulacion[06].x - 20, articulacion[04].y);
							SALTOwaist04b	= cv::Point(articulacion[12].x + 20, articulacion[04].y);		//	cintura
							//
							//SALTOcadera17 = cv::Point(articulacion[17].x - 20, articulacion[21].y);
							//SALTOcadera21 = cv::Point(articulacion[21].x + 20, articulacion[21].y);		//	cadera
							//			y los valores base en mm
							//SALTOhombroBasMM	= artFULL[06].real.y;						//	hombro base en mm
							//SALTOcaderaBasMM	= artFULL[17].real.y;						//	cadera base en mm

							SALTOwaistBasMM		= artFULL[04].real.y;						//	centro de masa base en mm
							WAISTminMM			= RedonI(SALTOwaistBasMM - CMumbralMM);		//	valor de la base menos un un umbral en MM para detectar que bajó
							WAISTdistanceMM		= artFULL[04].real.z;						//	distancia al cuál estaba el tipo 
						}
					}
					else {
						//						no estoy cumpliendo con la condición para estar en este estado,
						//						el usuario está TORCIDO derecha izquierda respecto a la cámara
						SALTOstatus		= 0;				
						SALTOcontadorS	= 0;		//	?
						CONTANDO		= false;	//	?
						SALTANDO		= false;	//	?
						SALTOindex		= 0;
						break;
					}			
					break;
					// #############################################################################################
				case 2:					//		listo para saltar, "grabando" --> pueden bajar y volver a subir las líneas
					if ((DIFhombrosCM < (2 * DIFumbralCM)) && (DIFcaderasCM < (2 * DIFumbralCM)) && (abs(WAISTdistanceMM - artFULL[4].real.z) < CMdistUmbMM) ) {
						//						si sigo bien orientado y si estoy dentro del umbral de distancia...
						//						comparando la distancia BASE con la actual en el CENTRO DE MASA (WAIST)
						//
						//						muestro hombros y caderas en negro
						Hueso(skeleton_mat, articulacion, 06, 12, _black, esqueleto_ancho + 3);	//	línea de hombros			
						Hueso(skeleton_mat, articulacion, 17, 21, _black, esqueleto_ancho + 3);	//	línea de caderas	
						//
						//cv::line(skeleton_mat, SALTOhombro06, SALTOhombro12, _yellow, 1);		//	hombros
						//cv::line(skeleton_mat, SALTOcadera17, SALTOcadera21, _yellow, 1);		//	cadera
						//
						//						cintura en amarillo, flecha verde de altura, y valor
						cv::line(skeleton_mat, SALTOwaist04a, SALTOwaist04b, _yellow, 1);		//	cintura
						
						cv::arrowedLine(skeleton_mat, SALTOwaist04a, cv::Point(SALTOwaist04a.x, articulacion[4].y), _fluo, 4);	

						SALTOcurrentWAISTmm = RedonI(1 * (-SALTOwaistBasMM + artFULL[04].real.y));
						//
						//						sólo para verificar, pongo los valores de los umbrales
						/*
						cv::putText(skeleton_mat, cv::format("%d", SALTOhombroBasMM), cv::Point(SALTOhombro06.x - 120, articulacion[06].y), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, _yellow, 1, cv::LINE_AA);
						cv::putText(skeleton_mat, cv::format("%d", SALTOcaderaBasMM), cv::Point(SALTOcadera17.x - 120, articulacion[17].y), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, _yellow, 1, cv::LINE_AA);
						cv::putText(skeleton_mat, cv::format("%d", SALTOwaistBasMM), cv::Point(SALTOwaist04a.x - 120, articulacion[04].y), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, _yellow, 1, cv::LINE_AA);
						cv::putText(skeleton_mat, cv::format("%d/%d", SALTOcurrentWAISTmm,WAISTminMM), cv::Point(articulacion[4].x - 120, articulacion[04].y-25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, _red, 1, cv::LINE_AA);
						*/
						//
						//						acá debiera esperar que el tipo se agache por debajo del límite,
						//						sinó, lo dejo estar... mientras que no se adelante y no se tuerza
						//						y ojo que en mm puede ser negativo...
						if (artFULL[04].real.y <= WAISTminMM) {
							//					si pasé por debajo del valor de umbral, significa que hace la sentadilla
							SALTOstatus = 3;	//	me voy al estado 3
							break;
						}
					}
					else {
						//	si el usuario está torcido o perdió distancia con los umbrales al doble. vuelvo al inicio.
						SALTOstatus		= 0;				
						SALTOcontadorS	= 0;
						CONTANDO		= false;
						SALTANDO		= false;
						SALTO			= 0;
						break;
					}			
					break;
					// #############################################################################################
				case 3:						//	está haciendo la sentadilla, pasó por debajo del valor de umbral
					//							en cuanto supere el valor de base arranca el salto
					//
					if ((DIFhombrosCM < (2 * DIFumbralCM)) && (DIFcaderasCM < (2 * DIFumbralCM)) && (abs(WAISTdistanceMM - artFULL[4].real.z) < CMdistUmbMM)) {
						//
						//						muestro hombros y caderas en negro
						Hueso(skeleton_mat, articulacion, 06, 12, _black, esqueleto_ancho + 3);	//	línea de hombros			
						Hueso(skeleton_mat, articulacion, 17, 21, _black, esqueleto_ancho + 3);	//	línea de caderas	
						//
						//						cintura en amarillo, flecha verde de altura, y valor
						cv::line(skeleton_mat, SALTOwaist04a, SALTOwaist04b, _yellow, 1);		//	cintura
						cv::arrowedLine(skeleton_mat, SALTOwaist04a, cv::Point(SALTOwaist04a.x, articulacion[4].y), _fluo, 4);
						SALTOcurrentWAISTmm = RedonI(1 * (-SALTOwaistBasMM + artFULL[04].real.y));
						//
						//						acá tengo que detectar cuando supera el valor del centro de masa original...
						//						significaría que está saltando, me voy al estado 4
						if ((artFULL[04].real.y > SALTOwaistBasMM)) {
							//					tengo que empezar a contar el tiempo de vuelo
							//					activar la bandera de saltando y pasar de estado...
							t_cur			= clock();
							CONTANDO		= true;
							SALTANDO		= true;
							SALTOstatus		= 4;
							SALTOindex		= 0;
							//					y detectar el máximo...
							SALTO			= 0;
							break;
						}
					}
					else {
						//					estoy torcido o me adelanté mucho, salgo
						SALTOstatus		= 0;				
						SALTOcontadorS	= 0;
						CONTANDO		= false;
						SALTANDO		= false;
						break;
					}
					break;
					// #######################################################################################
				case 4:						//	comenzó el salto. hay que guardar todos los datos
					//
					
					//						en principio no voy a controlar que esté torcido
					//						muestro hombros y caderas en negro
					Hueso(skeleton_mat, articulacion, 06, 12, _black, esqueleto_ancho + 3);	//	línea de hombros			
					Hueso(skeleton_mat, articulacion, 17, 21, _black, esqueleto_ancho + 3);	//	línea de caderas	
					//
					//						cintura en amarillo, flecha verde de altura, y valor
					cv::line(skeleton_mat, SALTOwaist04a, SALTOwaist04b, _yellow, 1);		//	cintura			
					cv::arrowedLine(skeleton_mat, SALTOwaist04a, cv::Point(SALTOwaist04a.x, articulacion[4].y), _fluo, 4);		
					//
					//						lo que salta es el valor del CM menos la base
					SALTOcurrentWAISTmm = RedonI(1 * (-SALTOwaistBasMM + artFULL[04].real.y));
					//	pongo en fluo el valor del salto
					cv::putText(skeleton_mat, cv::format("%d", SALTOcurrentWAISTmm), cv::Point(SALTOwaist04a.x - 120, articulacion[04].y), cv::FONT_HERSHEY_COMPLEX_SMALL, 2, _fluo, 1, cv::LINE_AA);

					//						pero si todavía no cae, guardo todo
					//						guardo el valor de diferencia de la cintura al valor marcado antes
					SALTOperfil[SALTOindex] = SALTOcurrentWAISTmm;		//	guardo el valor del salto
					SALTOindex++;										//	incremento el índice
					//
					if (SALTOindex > IM_ARRAYSIZE(SALTOperfil)) {		//
						SALTOindex = 0;
					}
					//
					//						voy sacando el máximo
					if (SALTOcurrentWAISTmm > SALTO) {
						SALTO				= SALTOcurrentWAISTmm;
						//				quiero mostrar el maximo con una línea roja y dejar los mm que saltó en rojo
						SALTOwaistMaxPX		= articulacion[4].y;
						SALTOwaist04aMax	= cv::Point(SALTOhombro06.x, SALTOwaistMaxPX);
						SALTOwaist04bMax	= cv::Point(SALTOhombro12.x, SALTOwaistMaxPX);
					}

					cv::line(skeleton_mat, SALTOwaist04aMax, SALTOwaist04bMax, _red, 1);	//
					cv::putText(skeleton_mat, cv::format("%d", SALTO), cv::Point(SALTOwaist04bMax.x + 20, SALTOwaist04bMax.y), cv::FONT_HERSHEY_COMPLEX_SMALL, 2.5, _red, 1, cv::LINE_AA);

					//
					//						tengo que guardar todos los valores hasta que vuelva a pasar por la posición de equilibrio
					if ((artFULL[04].real.y < SALTOwaistBasMM)) {
						//					cuando vuelve a pasar por el valor base, terminó el salto
						CONTANDO				= false;										//	
						SALTANDO				= false;										//	
						saltos[repeticiones]	= SALTO;										//	salto[repetición]
						SALTOtiempo				= (double(clock()) - double(t_cur)) / double(CLOCKS_PER_SEC);	//
						SALTOfin				= SALTOindex - 1;								//
						tiempos[repeticiones]	= SALTOtiempo;									//	tiempos[repetición]
						if (SALTO > SALTOrecord) {
							SALTOrecord			= SALTO;										//
							TIEMPOrecord		= SALTOtiempo;									//
						}
						repeticiones++;															//	
						SALTOstatus				= 1;											//	
						break;
					}
					//
					break;
					// #############################################################################################
					// #############################################################################################
		}
		}

		
		//	primeros intentos del salto en largo
//
		if (ACTIVIDADelegida == SALTO_LARGO) {
			//	vamos a registar los puntos del usuario, sin esqueleto por ahora
			//	la única articulación que me interesa detectar es el centro de masa, waist, la cintura, 4 porque el esqueleto va para atrás de perfil

			//	tengo círculo verde en WAIST con coordenadas (x,y) en MM
			cv::circle(skeleton_mat, articulacion[4],	4, _green,	-2);

			cv::putText(skeleton_mat, cv::format("(%d,%d)mm", userCMx_MM,	userCMy_MM),	articulacion[4],										cv::FONT_HERSHEY_COMPLEX_SMALL, 1, _black, 1, cv::LINE_AA);
			cv::putText(skeleton_mat, cv::format("(%d,%d)px", userCMx_PX ,	userCMy_PX ),	cv::Point(articulacion[4].x, articulacion[4].y + 30),	cv::FONT_HERSHEY_COMPLEX_SMALL, 1, _black, 1, cv::LINE_AA);

			cv::circle(skeleton_mat, userTopLeft,		4, _green,	-2);						//	círculos amarillos en rectángulo
			cv::circle(skeleton_mat, userTopRight,		4, _yellow, -2);						//	círculos amarillos en rectángulo
			cv::circle(skeleton_mat, userBottomLeft,	4, _green,	-2);						//	círculos amarillos en rectángulo
			cv::circle(skeleton_mat, userBottomRight,	4, _yellow, -2);						//	círculos amarillos en rectángulo		

			//	pasos 
			//	1: fijar el punto de partida estable (y CM en ese punto)
			//		me tiene que quedar 
			//				CM_inicial_PX
			//				CM_inicial_MM
			//				PUNTAS_inicial_PX --> que es userBottomRight si salta hacia la derecha
			//	2: esperar que salte (se agacha también)
			//	3: esperar que caiga y ahí obtener
			//				CM_final_PX
			//				CM_final_MM
			//				PUNTAS_final_PX --> que es userBottomRight si salta hacia la derecha
			//		con eso calculo la distancia en MM y en PX para convertir --> PXtoMM 
			//	4: con el valor final de us

			switch (SALTOstatus) {
				// #############################################################################################
			case 0:		//			centro de masa inestable --> punta del pie inestable.
				//
				CONTANDO = false;							//	temporizador off
				SALTANDO = false;							//	salto off
						
				SALTOcontadorS++;							//	incremento contador de estabilidad

				switch (SALTOcontadorS) {
				case 5:
					userCMx_MM1		= userBottomLeft.x;		//	centro de masa x píxels AUX 1
					userCMy_MM1		= userBottomLeft.y;
					break;
				case 13:
					userCMx_MM2		= userBottomLeft.x;		//	centro de masa x píxels AUX 1
					userCMy_MM2		= userBottomLeft.y;
					break;
				case 20:
					userCMx_MM3		= userBottomLeft.x;		//	centro de masa x píxels AUX 1
					userCMy_MM3		= userBottomLeft.y;
					SALTOcontadorS	= 0;
					break;
				}
				//					calculo la media de 3 mediciones seguidas
				mainCMxMM = (int)((userCMx_MM1 + userCMx_MM2 + userCMx_MM3) / 3);
				mainCMyMM = (int)((userCMy_MM1 + userCMy_MM2 + userCMy_MM3) / 3);
				//

				//					si el valor actual está en un entorno de la media de los últimos tres...
				if (( abs(userBottomLeft.x - mainCMxMM) < 30 ) && (abs(userBottomLeft.y - mainCMyMM) < 30)){					
					//				y aumento el contador para buscar la estabilidad
					SALTOcont++;		//
					if (SALTOcont > STABLEthr)
					{
						//			después de X veces, paso de estado
						SALTOstatus			= 1;
						SALTOcontadorS		= 0;
						//			y acá dejo marcado los puntos para construir las lineas fijas que serán la base del salto
						//			techo, piso y línea de caderas

						SALTOpisoPXa		= cv::Point(50,		userBottomLeft.y);
						SALTOpisoPXb		= cv::Point(750,	userBottomLeft.y);
						SALTOwaist04a		= cv::Point(50,		userCMy_PX);
						SALTOwaist04b		= cv::Point(750,	userCMy_PX);						//	cintura						
						//			y los valores base en mm
						SALTOwaistBasMM		= artFULL[04].real.y;									//	centro de masa base en mm

						//			ojo que en mm crece para arriba, y en pixels crece para abajo
						//			hay que tener mucho cuidad cuando se trabaja con el eje y porque dá negativo y positivo...
						WAISTminMM			= RedonI(SALTOwaistBasMM - CMumbralMM);					//
						WAISTdistanceMM		= artFULL[04].real.z;									//	distancia al cuál estaba el tipo 

						//			valores iniciales del centro de masa y las puntas de pié
						CM_inicial_PX		= cv::Point(userCMx_PX,			userCMy_PX);			//	centro de masa INICIAL en PIXELS
						CM_inicial_MM		= cv::Point(userCMx_MM,			userCMy_MM);			//	centro de masa INICIAL en MM
						PUNTAS_inicial_PX	= cv::Point(userBottomRight.x,	userBottomRight.y);		//	

					}
				}
				else {
					//						no estoy cumpliendo con la condición para estar en este estado,
					//						el usuario está TORCIDO derecha izquierda respecto a la cámara
					SALTOstatus		= 0;
					SALTOcont		= 0;		//	
					CONTANDO		= false;	//	?
					SALTANDO		= false;	//	?
					SALTOindex		= 0;
					break;
				}
				break;
				// #############################################################################################
			case 1:					//		centro de masa estable, tengo las 3 líneas marcadas
				//							tiene que bajar para hacer la sentadilla
				if ((abs(WAISTdistanceMM - userCMz_MM) < CMdistUmbMM)) {
					//						si no se adelanta o retira más del umbral...
					//
					//						muestro techo, piso y caderas
					//cv::line(skeleton_mat, SALTOtechoPXa, SALTOtechoPXb, _yellow, 1);		//	techo
					//cv::line(skeleton_mat, SALTOpisoPXa, SALTOpisoPXb, _cyan, 1);			//	piso
					cv::line(skeleton_mat, SALTOwaist04a, SALTOwaist04b, _yellow, 1);		//	cintura
					cv::line(skeleton_mat, SALTOpisoPXa, SALTOpisoPXb, _black, 1);			//	piso
					cv::line(skeleton_mat, cv::Point(PUNTAS_inicial_PX.x, 50), cv::Point(PUNTAS_inicial_PX.x, 400), _black, 1);		// inicio
					cv::circle(skeleton_mat, PUNTAS_inicial_PX, 4, _black, -2);				//	círculo negro inicial

																							//
					//						flecha verde de altura, y valor
					//cv::arrowedLine(skeleton_mat, PUNTAS_inicial_PX, cv::Point(userCMx_PX, PUNTAS_inicial_PX.y ), _fluo, 4);
					//SALTOcurrentWAISTmm = RedonI(1 * (-SALTOwaistBasMM + userCMy_MM));		//	valor del salto
					//
					//						sólo para verificar, pongo los valores de los umbrales
					/*
					cv::putText(skeleton_mat, cv::format("%d", SALTOhombroBasMM), cv::Point(SALTOhombro06.x - 120, articulacion[06].y), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, _yellow, 1, cv::LINE_AA);
					cv::putText(skeleton_mat, cv::format("%d", SALTOcaderaBasMM), cv::Point(SALTOcadera17.x - 120, articulacion[17].y), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, _yellow, 1, cv::LINE_AA);
					cv::putText(skeleton_mat, cv::format("%d", SALTOwaistBasMM), cv::Point(SALTOwaist04a.x - 120, articulacion[04].y), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, _yellow, 1, cv::LINE_AA);
					cv::putText(skeleton_mat, cv::format("%d/%d", SALTOcurrentWAISTmm,WAISTminMM), cv::Point(articulacion[4].x - 120, articulacion[04].y-25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, _red, 1, cv::LINE_AA);
					*/

					//
					//						acá debiera esperar que el tipo se agache por debajo del límite,
					//						sinó, lo dejo estar... mientras que no se adelante y no se tuerza	
					//						y ojo que en mm puede ser negativo...
					if (userCMy_MM <= WAISTminMM) {
						//					si pasé por debajo del valor de umbral, significa que hace la sentadilla
						SALTOstatus = 2;	//	me voy al estado 3
						break;
					}

				}
				else {
					//	si el usuario está torcido o perdió distancia con los umbrales al doble. vuelvo al inicio.
					SALTOstatus		= 0;
					//SALTOcont		= 0;
					SALTOcontadorS	= 0;
					CONTANDO		= false;
					SALTANDO		= false;
					break;
				}
				break;
				// #############################################################################################
			case 2:						//	está haciendo la sentadilla, pasó por debajo del valor de umbral
				//							en cuanto supere el valor de base arranca el salto
				//
				if ((abs(WAISTdistanceMM - userCMz_MM) < CMdistUmbMM)) {
					//						si no cambió la distancia...
					//						muestro techo, piso y caderas
					//cv::line(skeleton_mat, SALTOtechoPXa, SALTOtechoPXb, _yellow, 1);		//	techo
					//cv::line(skeleton_mat, SALTOpisoPXa, SALTOpisoPXb, _yellow, 1);			//	piso
					//cv::line(skeleton_mat, SALTOwaist04a, SALTOwaist04b, _yellow, 1);		//	cintura
					//
					cv::line(skeleton_mat, SALTOwaist04a,						SALTOwaist04b,							_yellow, 1);	//	cintura
					cv::line(skeleton_mat, SALTOpisoPXa,						SALTOpisoPXb,							_black,  1);	//	piso
					cv::line(skeleton_mat, cv::Point(PUNTAS_inicial_PX.x, 50),	cv::Point(PUNTAS_inicial_PX.x, 400),	_black,  1);	//	inicio

					cv::circle(skeleton_mat, PUNTAS_inicial_PX, 4, _black, -2);				//	círculo negro inicial

					//						flecha verde de altura, y valor
					cv::arrowedLine(skeleton_mat, PUNTAS_inicial_PX, cv::Point(userCMx_PX, PUNTAS_inicial_PX.y), _fluo, 4);

					SALTOcurrentWAISTmm = RedonI(1 * (-SALTOwaistBasMM + userCMy_MM));		//	valor del salto
					//
					//						acá tengo que detectar cuando supera el valor del centro de masa original...
					//						significaría que está saltando, me voy al estado 4					
					if ((userCMy_MM > SALTOwaistBasMM)) {
						//					no estoy seguro de que tenga que ser esta valor...
						//					tengo que empezar a contar el tiempo de vuelo
						//					activar la bandera de saltando y pasar de estado...
						t_cur		= clock();
						CONTANDO	= true;
						SALTANDO	= true;
						SALTOstatus = 3;
						SALTOindex	= 0;
						//					y detectar el máximo...
						SALTO		= 0;
						break;
					}
				}
				else {
					//					estoy torcido o me adelanté mucho, salgo
					SALTOstatus		= 0;
					SALTOcontadorS	= 0;
					CONTANDO		= false;
					SALTANDO		= false;
					break;
				}
				break;
				// #######################################################################################
			case 3:						//	comenzó el salto. hay que guardar todos los datos
				//
				//						muestro techo, piso y caderas
				//cv::line(skeleton_mat, SALTOtechoPXa, SALTOtechoPXb, _yellow, 4);		//	techo
				//cv::line(skeleton_mat, SALTOpisoPXa, SALTOpisoPXb, _yellow, 4);			//	piso
				//cv::line(skeleton_mat, SALTOwaist04a, SALTOwaist04b, _yellow, 4);		//	cintura
				//
				cv::line(skeleton_mat, SALTOwaist04a,						SALTOwaist04b,							_yellow, 1);	//	cintura
				cv::line(skeleton_mat, SALTOpisoPXa,						SALTOpisoPXb,							_black,  1);	//	piso
				cv::line(skeleton_mat, cv::Point(PUNTAS_inicial_PX.x, 50),	cv::Point(PUNTAS_inicial_PX.x, 400),	_black,  1);	// inicio

				cv::circle(skeleton_mat, PUNTAS_inicial_PX, 4, _black, -2);				//	círculo negro inicial

				//						flecha verde de altura, y valor
				cv::arrowedLine(skeleton_mat, PUNTAS_inicial_PX, cv::Point(userCMx_PX, PUNTAS_inicial_PX.y), _fluo, 4);

				SALTOcurrentWAISTmm = RedonI(1 * (-SALTOwaistBasMM + userCMy_MM));		//	valor del salto vertical
				//
				//	pongo en fluo el valor del salto
				//cv::putText(skeleton_mat, cv::format("%d", SALTOcurrentWAISTmm), cv::Point(userCMx_PX - 120, userCMy_PX), cv::FONT_HERSHEY_COMPLEX_SMALL, 2, _fluo, 1, cv::LINE_AA);

				//						pero si todavía no cae, guardo todo
				//						guardo el valor de diferencia de la cintura al valor marcado antes
				SALTOperfil[SALTOindex]		= SALTOcurrentWAISTmm;								//	guardo el valor del salto
				CMsaltoLargoX[SALTOindex]	= userCMx_PX;										//
				CMsaltoLargoY[SALTOindex]	= userCMy_PX;										//
				SALTOindex++;																	//	incremento el índice
				cv::circle(skeleton_mat, cv::Point(userCMx_PX, userCMy_PX), 8, _fucsia, -2);	//	círculo fucsia
				//
				//
				if (SALTOindex > IM_ARRAYSIZE(SALTOperfil)) {		//
					SALTOindex = 0;
				}
				//				
				//						tengo que guardar todos los valores hasta que vuelva a pasar por la posición de equilibrio				
				if ((userCMy_MM < SALTOwaistBasMM)) {
					//					cuando vuelve a pasar por el valor base, dejo de contar el tiempo, pero espero por la posición de equilibrio...
					CONTANDO				= false;
					SALTANDO				= false;
					SALTOtiempo				= (double(clock()) - double(t_cur)) / double(CLOCKS_PER_SEC);
					SALTOfin				= SALTOindex - 1;					
					SALTOstatus				= 4;
					SALTOcont				= 0;
					SALTOcontadorS			= 0;
					break;
				}
				//
				if ((abs(WAISTdistanceMM - userCMz_MM) > CMdistUmbMM)) {
					SALTOstatus		= 0;
					SALTOcontadorS	= 0;					
					CONTANDO		= false;
					SALTANDO		= false;
					break;
				}
				break;
				// #############################################################################################
			case 4:
				//				
				//						muestro techo, piso y caderas
				//cv::line(skeleton_mat, SALTOtechoPXa, SALTOtechoPXb, _yellow, 1);		//	techo
				
				//cv::line(skeleton_mat, SALTOpisoPXa, SALTOpisoPXb, _yellow, 1);			//	piso
				//cv::line(skeleton_mat, SALTOwaist04a, SALTOwaist04b, _yellow, 3);		//	cintura
				
																						
				cv::line(skeleton_mat, SALTOwaist04a,						SALTOwaist04b,							_yellow, 1);	//	cintura
				cv::line(skeleton_mat, SALTOpisoPXa,						SALTOpisoPXb,							_black, 1);		//	piso
				cv::line(skeleton_mat, cv::Point(PUNTAS_inicial_PX.x, 50),	cv::Point(PUNTAS_inicial_PX.x, 400),	_black, 1);		// inicio

				cv::circle(skeleton_mat, PUNTAS_inicial_PX, 4, _black, -2);				//	círculo negro inicial

				//						grafico el movimiento del centro de masa
				//for (int jj = 0; jj < SALTOfin; jj++) {
				//	cv::circle(skeleton_mat, cv::Point(CMsaltoLargoX[jj], CMsaltoLargoY[jj]), 8, _fucsia, -2);									//	círculo fucsia
				//}
				//
				SALTOcontadorS++;				//	
				//								//	ahora necesito estabilidad del punto de llegada
				/*
				switch (SALTOcontadorS) {
				case 5:
					userCMx_MM1 = userCMx_MM;	//	centro de masa x píxels AUX 1
					userCMy_MM1 = userCMy_MM;
					break;
				case 13:
					userCMx_MM2 = userCMx_MM;	//	centro de masa x píxels AUX 1
					userCMy_MM2 = userCMy_MM;
					break;
				case 20:
					userCMx_MM3 = userCMx_MM;	//	centro de masa x píxels AUX 1
					userCMy_MM3 = userCMy_MM;
					SALTOcontadorS = 0;
					break;
				}
				//					calculo la media de 3 mediciones seguidas
				mainCMxMM = (int)((userCMx_MM1 + userCMx_MM2 + userCMx_MM3) / 3);
				mainCMyMM = (int)((userCMy_MM1 + userCMy_MM2 + userCMy_MM3) / 3);
				//
				//					si el valor actual está en un entorno de la media de los últimos tres...
				if ( ( abs(userCMx_MM - mainCMxMM) < 40 ) && ( abs( userCMy_MM - mainCMyMM ) < 40 ) ) {
				*/

				switch (SALTOcontadorS) {
				case 5:
					userCMx_MM1		= userBottomRight.x;	//	centro de masa x píxels AUX 1
					userCMy_MM1		= userBottomRight.y;
					break;
				case 13:
					userCMx_MM2		= userBottomRight.x;	//	centro de masa x píxels AUX 1
					userCMy_MM2		= userBottomRight.y;
					break;
				case 20:
					userCMx_MM3		= userBottomRight.x;	//	centro de masa x píxels AUX 1
					userCMy_MM3		= userBottomRight.y;
					SALTOcontadorS	= 0;
					break;
				}
				//					calculo la media de 3 mediciones seguidas
				mainCMxMM = (int)((userCMx_MM1 + userCMx_MM2 + userCMx_MM3) / 3);
				mainCMyMM = (int)((userCMy_MM1 + userCMy_MM2 + userCMy_MM3) / 3);
				//
				//					si el valor actual está en un entorno de la media de los últimos tres...
				if ((abs(userBottomRight.x - mainCMxMM) < 40) && (abs(userBottomRight.y - mainCMyMM) < 40)) {					
					//				y aumento el contador para buscar la estabilidad
					SALTOcont++;		//
					if (SALTOcont > STABLEthr)
					{
						//			después de X veces, paso de estado
						SALTOstatus		= 5;
						SALTOcontadorS	= 0;

						
						//			valores iniciales del centro de masa y las puntas de pié
						CM_final_PX		= cv::Point(userCMx_PX,			userCMy_PX);			//	centro de masa FINAL en PIXELS
						CM_final_MM		= cv::Point(userCMx_MM,			userCMy_MM);			//	centro de masa FINAL en MM
						PUNTAS_final_PX	= cv::Point(userBottomLeft.x,	userBottomLeft.y);		//
						
						//			ahora que tengo los valores finales,puedo calcular la distancia del salto en MM
						//			1) calculo la distancia en píxels del centro de masa

						float		CMdistancia_PX	= RedonD(sqrt(pow(CM_final_PX.x - CM_inicial_PX.x, 2) + pow(CM_final_PX.y - CM_inicial_PX.y, 2)));
						float		CMdistancia_MM	= RedonD(sqrt(pow(CM_final_MM.x - CM_inicial_MM.x, 2) + pow(CM_final_MM.y - CM_inicial_MM.y, 2)));
						float		CMescala		= RedonD(CMdistancia_MM / CMdistancia_PX);
						float		CMsalto_PX		= RedonD(sqrt(pow(PUNTAS_final_PX.x - PUNTAS_inicial_PX.x, 2) + pow(PUNTAS_final_PX.y - PUNTAS_inicial_PX.y, 2)));

						SALTOlargoMM	= RedonI(CMescala * CMsalto_PX);	//
						SALTO			= SALTOlargoMM;						//
						//						actualizo tiempos
						saltos[repeticiones]	= SALTOlargoMM;				//
						tiempos[repeticiones]	= SALTOtiempo;				//
						if (SALTO > SALTOrecord) {
							SALTOrecord			= SALTO;					//
							TIEMPOrecord		= SALTOtiempo;				//
						}
						repeticiones++;

					}
				}
				else {
					SALTOcont = 0;
					//						no estoy cumpliendo con la condición para estar en este estado,		
					if ((abs(WAISTdistanceMM - userCMz_MM) > CMdistUmbMM)) {
						SALTOstatus		= 0;
						SALTOcontadorS	= 0;
						CONTANDO		= false;
						SALTANDO		= false;
						break;
					}
					/*
					if (SALTOcontadorS > 2000) {
						SALTOstatus = 0;
						SALTOcont = 0;		//	
						CONTANDO = false;	//	?
						SALTANDO = false;	//	?
						SALTOindex = 0;
					}
					break;
					*/
				}
				break;
				// #############################################################################################
				case 5:
					

					//						muestro techo, piso y caderas
					////cv::line(skeleton_mat, SALTOtechoPXa, SALTOtechoPXb, _yellow, 1);		//	techo
					//cv::line(skeleton_mat, SALTOpisoPXa, SALTOpisoPXb, _yellow, 1);			//	piso
					//cv::line(skeleton_mat, SALTOwaist04a, SALTOwaist04b, _yellow, 1);		//	cintura
					//
					cv::line(skeleton_mat,	SALTOwaist04a,							SALTOwaist04b,							_yellow,	1);		//	cintura
					cv::line(skeleton_mat,	SALTOpisoPXa,							SALTOpisoPXb,							_black,		1);		//	piso
					cv::line(skeleton_mat,	cv::Point(PUNTAS_inicial_PX.x, 50),		cv::Point(PUNTAS_inicial_PX.x, 400),	_black,		1);		// inicio

					cv::circle(skeleton_mat,	PUNTAS_inicial_PX,	4, _black, -2);					//	círculo negro inicial

					cv::line(skeleton_mat,	cv::Point(PUNTAS_final_PX.x, 50),		cv::Point(PUNTAS_final_PX.x, 400),		_fucsia,	1);		// inicio

					cv::circle(skeleton_mat,	PUNTAS_final_PX,	4, _fucsia, -2);				//	círculo negro inicial

					//
					cv::arrowedLine(skeleton_mat, PUNTAS_inicial_PX, PUNTAS_final_PX, _fluo, 4);

					cv::putText(skeleton_mat, cv::format("%d", SALTOlargoMM), cv::Point(userCMx_PX-50, PUNTAS_inicial_PX.y-50), cv::FONT_HERSHEY_COMPLEX_SMALL, 2, _fluo, 1, cv::LINE_AA);

					//						grafico el movimiento del centro de masa

					//						círculo inicial
					cv::circle(skeleton_mat, CM_inicial_PX,		8, _green, -2);
					cv::circle(skeleton_mat, PUNTAS_inicial_PX, 8, _black, -2);
					//						círculos del salto (CM)
					for (int jj = 0; jj < SALTOfin; jj++) {

						cv::circle(skeleton_mat, cv::Point(CMsaltoLargoX[jj], CMsaltoLargoY[jj]), 8, _fucsia, -2);									//	círculo fucsia
					}
					//						círculo final
					cv::circle(skeleton_mat, CM_final_PX,		8, _red, -2);
					cv::circle(skeleton_mat, PUNTAS_final_PX,	8, _red, -2);

					//
					if ((abs(WAISTdistanceMM - userCMz_MM) > CMdistUmbMM)) {
						SALTOstatus		= 0;
						SALTOcontadorS	= 0;
						CONTANDO		= false;
						SALTANDO		= false;
						break;
					}


					break;
				// #############################################################################################
			}
		}
	}
	cv::resize(skeleton_mat, skeleton_mat, cv::Size(final_width, final_height ), 0, 0, 1);

}
// fin de skeleton *****************************

// Show Data
void NuiTrack::show()
{
    // Show User
    //showUser();
	//*****************************
	// Show Skeleton
	showSkeleton();
	//*****************************

}


// Show User
/*
inline void NuiTrack::showUser()
{
    if( user_mat.empty() ){
        return;
    }

    // Show User Image
    cv::imshow( "User", user_mat );
}
*/

// deskeleton
// Show Skeleton
inline void NuiTrack::showSkeleton()
{
	if (skeleton_mat.empty()) {
		//return;
	}

	cv::namedWindow("INAUT PDTS 2019: Detección y medición de movimiento", cv::WINDOW_NORMAL);
	//  alto y ancho deseado de la pantalla final
	int Pancho_des = 1600;	// 1390;	// 1856;
	int	Palto_des  = 650;	// 522;	// 696;
	cv::resizeWindow("INAUT PDTS 2019: Detección y medición de movimiento", cv::Size(Pancho_des, Palto_des));	//es para verla el 50% mejor

	cv::moveWindow("INAUT PDTS 2019: Detección y medición de movimiento", (int)((1600 - Pancho_des) / 2), (int)((900 - Palto_des) / 2));
	//cv::moveWindow("INAUT PDTS 2019: Detección y medición de movimiento", (int)((1440 - Pancho_des) / 2), (int)((705 - Palto_des) / 2));
	//cv::moveWindow("INAUT PDTS 2019: Detección y medición de movimiento", (int)((1920-Pancho_des)/2), (int)((940 - Palto_des) / 2));

	// Show Skeleton Image
	cv::imshow("INAUT PDTS 2019: Detección y medición de movimiento", skeleton_mat);	
	
	
}


//Arco interior entre los puntos j1, j2 y j3
//* imagen mat
//* índice joint1
//* índice joint2
//* índice joint3
//* color
//* ancho (-1 para rellena 100 para no mostrar)
double Arco(cv::Mat esqueleto, int joint1, int joint2, int joint3, cv::Scalar color, int ancho)
{
	cv::Point			Paux = cv::Point(0, 0);		//  auxiliar
	std::string			str;
	std::ostringstream	angle;

	double		ANGi		= 0.0;		//  ángulo inicial
	double		ANGf		= 0.0;		//  ángulo final
	double		ANGrot		= 0.0;		//  rotación de la elipse
	double		ANGdelta	= 0.0;		//  el error 
	double		ANGdelta2	= 0.0;		//  el error en 3D
	double		ANGaux		= 0.0;		//  ángulo auxiliar
	double		ANGaux2		= 0.0;		//  ángulo auxiliar

	//  descartamos que estén los tres puntos. si alguno es cero salimos sin dibujar
	if ((articulacion[joint1].x == 0) || (articulacion[joint2].x == 0) || (articulacion[joint3].x == 0)) {
		return -1;
	}

	//	ángulos auxiliares para dibujar la elipse
	ANGaux		= Ver360( 180 * atan2(articulacion[joint1].y - articulacion[joint2].y, articulacion[joint1].x - articulacion[joint2].x) / 3.14 );	//saco los ang con cada hueso
	ANGaux2		= Ver360( 180 * atan2(articulacion[joint3].y - articulacion[joint2].y, articulacion[joint3].x - articulacion[joint2].x) / 3.14 );
	ANGdelta	= Ver360( ANGaux - ANGaux2);

	// elipse	
	if (ANGdelta > 180) {						// significa que aux > aux2 en más de 180º
		ANGdelta = 360 - ANGdelta;				// tomo el suplementario
		ANGrot	= ANGaux;						// roto la elipse hasta el final (aux)
		ANGi	= 0;							// y voy desde 0
		ANGf	= ANGdelta;						// hasta ANGdelta
	}
	else {
		if (ANGdelta > 0) {						// significa que aux > aux2 en menos de 180º
			ANGrot	= ANGaux2;					// roto la elipse hasta el inicio (aux2)
			ANGi	= 0;						// y voy desde 0
			ANGf	= ANGdelta;					// hasta ANGdelta
		}
		else {
			if (ANGdelta < -180) {				// aux2 > aux en más de 180º
				ANGdelta = 360 + ANGdelta;		// tomo el suplementario
				ANGrot	= ANGaux2;				// roto la elipse hasta el final (aux2)
				ANGi	= 0;					// y voy desde 0
				ANGf	= ANGdelta;				// hasta ANGdelta
			}
			else {								// aux2 > aux en menos de 180º
				ANGrot	= ANGaux;				// roto la elipse hasta el inicio (aux)
				ANGi	= 0;					// y voy desde 0
				ANGf	= abs(ANGdelta);		// hasta ANGdelta
			}
		}
	}

	// MÉTODO CORTO de cálculo del ángulo en 3D
	// 1) calculo los dos vectores
	int ux, uy, uz, vx, vy, vz;
	double u_norm, v_norm;

	if ((artFULL[joint1].confidence < 0.3) || (artFULL[joint2].confidence < 0.3) || (artFULL[joint3].confidence < 0.3)) {
		return ANGdelta;
	}

	ux = artFULL[joint2].real.x - artFULL[joint1].real.x;
	uy = artFULL[joint2].real.y - artFULL[joint1].real.y;
	uz = artFULL[joint2].real.z - artFULL[joint1].real.z;
	vx = artFULL[joint2].real.x - artFULL[joint3].real.x;
	vy = artFULL[joint2].real.y - artFULL[joint3].real.y;
	vz = artFULL[joint2].real.z - artFULL[joint3].real.z;

	u_norm		= sqrt(pow(ux, 2) + pow(uy, 2) + pow(uz, 2));
	v_norm		= sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
	ANGdelta2	= acos(((float)(ux * vx) + (float)(uy * vy) + (float)(uz * vz)) / u_norm / v_norm) * 180 / 3.141519;

		

	//angle << std::fixed;
	//angle << std::setprecision(2);
	//angle << ANGdelta;

	//str = angle.str();
	
	if (ancho == 100) {
		return ANGdelta2;		//si puso ancho cero es que no quiere verlo
	}

	const int	ANCHO_elipse = round(30 * 1000 / artFULL[joint2].real.z);
	cv::ellipse(esqueleto, articulacion[joint2], cv::Size(ANCHO_elipse, ANCHO_elipse), ANGrot, ANGi, ANGf, color, ancho, 8);		//ancho 3
	//cv::ellipse(esqueleto, articulacion[joint2], cv::Size(30, 30), ANGrot, ANGi, ANGf, color, ancho, 8);		//ancho 3

	//	si no se especifica el ancho (thichness) con un valor positivo, entonces es rellena
	//cv::ellipse(esqueleto, articulacion[joint2], cv::Size(30, 30), ANGrot, ANGi, ANGf, color, -1, 8); //esa es rellena
		
	// es un tema la ubicación de la etiqueta
	Paux.x = articulacion[joint2].x + 15;
	Paux.y = articulacion[joint2].y + 15;
	//cv::putText(esqueleto, str, Paux, cv::FONT_HERSHEY_TRIPLEX, 0.7, color, 1);
	cv::putText(esqueleto, cv::format( "%4.0f",RedonD(ANGdelta2) ), Paux, cv::FONT_HERSHEY_TRIPLEX, 0.4, color, 1);
	
	return ANGdelta;
}


//hueso
void	Hueso(cv::Mat esqueleto, cv::Point arts[25], int joint1, int joint2, cv::Scalar color, int ancho) {
	//	hola como te va
	if ((arts[joint1] == cv::Point(0, 0)) || (arts[joint2] == cv::Point(0, 0)))
		return;	
	cv::line(esqueleto, arts[joint1], arts[joint2], color, ancho);	//dibujo solamente si ninguno es cero
}

//Ver360
double Ver360(double ang) {	
	while (ang >= 360) {
		ang -= 360;
	}
	while (ang <= -360) {
		ang += 360;
	}
	if (ang < 0) {
		ang = 360 + ang;
	}
	return ang;
}

// filtro alpha beta
double filtroAB(double x_obs, double x_old, double &vx_old)
{
	
	double xEst;
	double vx;
	double x;

	//Prediction
	xEst	= x_old +  vx_old / FPSrealsense;		//estimado

	//Filtering
	x		= (xEst + ALFA * (x_obs - xEst) );

	vx		= vx_old + ((BETA * FPSrealsense) * (x_obs - xEst));

	//Actualizacion de valores 
	x_old	= x;
	vx_old	= vx;
	
	return x; 
	//return 0;
}


double RedonD(double aux) {
	aux		= 0.1 * (int)(10.0 * aux);
	return aux;
}

int RedonI(int aux) {
	aux		= (int) (0.1 * (int)(10.0 * aux) );
	return aux;
}


// Helper to display a little (?) mark which shows a tooltip when hovered.
// In your own code you may want to display an actual icon if you are using a merged icon fonts (see docs/FONTS.txt)
static void HelpMarker(const char* desc)
{
	ImGui::TextDisabled("(?)");
	if (ImGui::IsItemHovered())
	{
		ImGui::BeginTooltip();
		ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
		ImGui::TextUnformatted(desc);
		ImGui::PopTextWrapPos();
		ImGui::EndTooltip();
	}
}


void limpiar_records(void) {
	repeticiones	= 0;				//	limpio número de saltos
	SALTOrecord		= 0;				//	record altura
	TIEMPOrecord	= 0;				//	record tiempo
	//	limpio records
	for (int jmt = 0; jmt < ARRAYSIZE(saltos); jmt++) {
		saltos[jmt]		= 0.0;			//	los histogramas de abajo
		tiempos[jmt]	= 0.0;			//	
	}

}
