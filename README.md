# Control para robot háptico mediante OpenHaptics
Este *template* provee una forma sencilla de subir un controlador a un robot háptico que funcione bajo la plataforma *OpenHaptics*. Las configuraciones del entorno de programación ya se encuentran incluidas dentro del proyecto, además de que viene precargado con un ejemplo con control *PID*.

# Requerimientos
* [Touch Smart Driver](https://support.3dsystems.com/s/article/Haptic-Device-Drivers?language=en_US) (la opción que dice OpenHaptics v3.5)
* [Phantom Device Driver](https://support.3dsystems.com/s/article/Haptic-Device-Drivers?language=en_US) (inmediatamente abajo del anterior)
* [OpenHaptics Developer Edition](https://support.3dsystems.com/s/article/OpenHaptics-for-Windows-Developer-Edition-v35?language=en_US) (los dos drivers anteriores se encuentran en esta página también)
* [Visual Studio 2022](https://visualstudio.microsoft.com/es/vs/community/)
  * Desktop development for C++
  * C++ MFC Latest
  * C++ ATL Latest

# Instalación
## OpenHaptics
Solamente denle doble *clic* a los instaladores que descargaron, en el caso del *.zip* de *OpenHaptics*, descompriman el archivo y abran el instalador que resulta de la descompresión.

**Asegurense que la librería de OpenHaptics se haya añadido a sus variables de entorno de Windows.** Lo pueden poniendo la palabra *path* en el buscador de Windows. Se abrirá una ventana, denle *clic* al botón que dice **variables de entorno**. En la parte inferior, llamada **variables del sistema**, se debe de encontrar la variable ***OH_SDK_BASE***, su valor debe de ser `C:\OpenHaptics\Developer\3.5.0`. Si no existe dicha variable, la tienen que añadir ustedes manualmente.

## Visual Studio
1. Seleccionar la opción ***Desktop development with C++***
2. Seleccionar las siguientes opciones en el panel del lado derecho
  * MSVC v143 - VS 2022 C++ x64/x86 build tools
  * C++ ATL for latest v143 build tools (x86 & x64)
  * C++ MFC for latest v143 build tools (x86 & x64)
  * C++ Modules for v143 build tools (x64/x86)
  * Windows App SDK C++ Templates
  * **En el caso de tener problemas, instalar las versiones 140, 141 y 142 de MSVC, se encuentran en la misma pestaña**
3. Darle a instalar (si tienen una conexión de internet lenta, pueden cambiar la opción de *install while downloading* a *download first*

# Utilizar el *template* y cargar el controlador al robot
1. Lo más sencillo es clonar el repositorio desde *github* utilizando *Visual Studio*. Para esto, abrimos *Visual Studio* y le damos *clic* en **Clonar repositorio**.
2. En el campo de texto ingresarán la dirección URL que aparece en el botón ***Code*** que está dentro de este mismo repositorio al inicio. Deberán ingresar el link que se encuentra en la pestaña ***HTTPS***.
3. Entrarán a *Visual Studio* inmediatamente después de darle a clonar, en la pantalla que aparece en *Visual Studio* abrán el archivo ***.sln***
4. Si hicieron todo bien, y no se desconfiguró nada deberían ser capaces de darle al botón de *play* donde dice *Local Windows Debugger*, las primeras veces se tardará un poquito en compilar, sean pacientes. El *template* viene precargado con un controlador *PID* muy sencillo, por lo qué pueden probar su instalación corriendo el programa y haciendo *clic* en los botones del programa en el siguiente orden:

$$
\text{Inicializar dispositivo } \rightarrow \text{ Calibracion } \rightarrow \text{ Lectura encoders } \rightarrow { Home } \rightarrow { SMC} 
$$

Esperen un poco de tiempo a que el robot llegue a *Home* antes de darle al botón *SMC*. 
**¡Lo lograrón! Happy hacking! :)**

# Código
El código central a modificar se encuentra dentro de los archivos `HapticTemplateDlg.cpp`, `HelperFunctions.cpp`. Dentro del primer archivo se encuentra el código de llamada del controlador del robot, el único método para editar que control se está utilizando es `CHapticTemplateDlg::SmcTimerProc()`, los demás son necesarios para el funcionamiento de la interfaz gráfica, inicialización, calibración, entre otros. Dentro de `HelperFunctions.cpp` se encuentran funciones auxiliares para el cálculo de diferentes valores necesarios, como derivadas de Levant, etcétera. Tl;dr: si van a cambiar el controlador, muevan `SmcTimerProc()`, y auxiliense con `HelperFunctions.cpp`.

Suerte :)
