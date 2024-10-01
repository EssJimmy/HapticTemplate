// HapticTemplateDlg.cpp : implementation file
// TODO: memory revised leaks were reduced but now need to rework graphing method
#include "pch.h"
#include "HapticTemplateDlg.h"
#include "Controllers.h"
#include "afxdialogex.h"
#include <mmsystem.h>
#include <fstream>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <chrono>
#include <ctime>
#include <string>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif // DEBUG 

// Value inside needs to store different values
// but struct itself needs to be a pointer, thus needing
// this weird arrangement
typedef struct {
				hduVector3Dd position;
} device_state_struct;

// constant values for calculations and other stuff
constexpr int no_joints = 3;
constexpr double sample_time = 0.001;
constexpr double pi = 3.1415926535;

// non constant global values needed in several functions
double qm[no_joints] = { 0.0 };
std::vector<double> taum(no_joints, 0.0);

// file for saving data
std::string file_name = "data";
std::time_t end_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
struct tm datetime;
auto err = localtime_s(&datetime, &end_time);
char output[50];
auto x = strftime(output, sizeof output, "%Y%b%d-%H%M%S", &datetime);
char buffer[50];
auto n = sprintf_s(buffer, "./graph_data/data%s.csv", output);
std::ofstream graph_file(buffer);

// haptic robot stuff
HHD hHDm;
HDSchedulerHandle servo_loop_handle;
device_state_struct state;

//Timers
MMRESULT home_timer_id;
MMRESULT smc_timer_id;

//flags
bool initialized = false, scheduler_started = false;
bool i_c_home = true, home_flag = true, home_completed_flag = true;
bool i_c_smc = true, smc_flag = true, smc_completed_flag = true;
bool completed = true;

// CAboutDlg dialog used for App About
class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
 void DoDataExchange(CDataExchange* p_dx) override;    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* p_dx)
{
	   CDialogEx::DoDataExchange(p_dx);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CHapticTemplateDlg dialog
CHapticTemplateDlg::CHapticTemplateDlg(CWnd* p_parent /*=nullptr*/)
	: CDialogEx(IDD_HAPTICTEMPLATE_DIALOG, p_parent)
{
	   m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CHapticTemplateDlg::DoDataExchange(CDataExchange* p_dx)
{
	    CDialogEx::DoDataExchange(p_dx);
	    DDX_Control(p_dx, IDC_ENCODER4, m_time);
	    DDX_Control(p_dx, IDC_ENCODER1, m_Encoder1);
	    DDX_Control(p_dx, IDC_ENCODER5, m_statusTextBox);
	    DDX_Control(p_dx, IDC_ENCODER2, m_Encoder2);
	    DDX_Control(p_dx, IDC_ENCODER3, m_Encoder3);
}

BEGIN_MESSAGE_MAP(CHapticTemplateDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
				ON_BN_CLICKED(IDC_INITIALIZE, &CHapticTemplateDlg::on_bn_clicked_initialize)
				ON_BN_CLICKED(IDC_CALIB, &CHapticTemplateDlg::on_bn_clicked_calibration)
				ON_BN_CLICKED(IDC_REED, &CHapticTemplateDlg::on_bn_clicked_read)
				ON_BN_CLICKED(IDC_PID, &CHapticTemplateDlg::on_bn_clicked_home)
				ON_BN_CLICKED(IDC_SMC, &CHapticTemplateDlg::on_bn_clicked_smc)
END_MESSAGE_MAP()


// CHapticTemplateDlg message handlers

BOOL CHapticTemplateDlg::OnInitDialog()
{
	    CDialogEx::OnInitDialog();

	    // Add "About..." menu item to system menu.

	    // IDM_ABOUTBOX must be in the system command range.
	    ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	    ASSERT(IDM_ABOUTBOX < 0xF000);

	    CMenu* p_sys_menu = GetSystemMenu(FALSE);
	    if (p_sys_menu != nullptr)
	    {
					    CString str_about_menu;
	        const BOOL b_name_valid = str_about_menu.LoadString(IDS_ABOUTBOX);
		    
		    ASSERT(b_name_valid);
		    if (!str_about_menu.IsEmpty())
		    {
			    p_sys_menu->AppendMenu(MF_SEPARATOR);
			    p_sys_menu->AppendMenu(MF_STRING, IDM_ABOUTBOX, str_about_menu);
		    }
	    }

	    // Set the icon for this dialog.  The framework does this automatically
	    //  when the application's main window is not a dialog
	    SetIcon(m_hIcon, TRUE);			// Set big icon
	    SetIcon(m_hIcon, FALSE);		// Set small icon

	    // TODO: Add extra initialization here

	    return TRUE;  // return TRUE  unless you set the focus to a control
}

void CHapticTemplateDlg::OnSysCommand(UINT n_id, LPARAM l_param)
{
	    if ((n_id & 0xFFF0) == IDM_ABOUTBOX)
	    {
		    CAboutDlg dlgAbout;
		    dlgAbout.DoModal();
	    }
	    else
	    {
		    CDialogEx::OnSysCommand(n_id, l_param);
	    }
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.
void CHapticTemplateDlg::OnPaint()
{
	    if (IsIconic())
	    {
		        CPaintDC dc(this); // device context for painting

		        SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		        // Center icon in client rectangle
						    const int cx_icon = GetSystemMetrics(SM_CXICON);
	        const int cy_icon = GetSystemMetrics(SM_CYICON);
		        CRect rect;
		        GetClientRect(&rect);
		        const int x = (rect.Width() - cx_icon + 1) / 2;
		        const int y = (rect.Height() - cy_icon + 1) / 2;

		        // Draw the icon
		        dc.DrawIcon(x, y, m_hIcon);
	    }
	    else
	    {
    		    CDialogEx::OnPaint();
	    }
}

void CHapticTemplateDlg::OnClose() {
				timeEndPeriod(1);
				if (!smc_flag || !home_flag) {
								timeKillEvent(smc_timer_id);
								completed = smc_flag = home_flag = true;
				}

				if (initialized && hdIsEnabled(HD_FORCE_OUTPUT))
								hdDisable(HD_FORCE_OUTPUT);

				hdUnschedule(servo_loop_handle);
				if (scheduler_started) hdStopScheduler();

				if (initialized) hdDisableDevice(hHDm);

				graph_file << std::endl;
				graph_file.close();
				exit(0);
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CHapticTemplateDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


// Callback to calibration parameters established by the drivers
static HDCallbackCode HDCALLBACK CalibrationStatusCallback(void* p_user_data)
{
    const auto p_status = static_cast<HDenum*>(p_user_data);

				hdBeginFrame(hHDm);
				hdUpdateCalibration(HD_CALIBRATION_INKWELL);
				*p_status = hdCheckCalibration();
				hdEndFrame(hHDm);

				return HD_CALLBACK_DONE;
}

// How the robot moves, by using a standard defined torque and the taum calculated in the
// SmcTimerProc method
static HDCallbackCode HDCALLBACK ServoLoopCallback(void* p_user_data) {
				const auto p_state = static_cast<device_state_struct*>(p_user_data);
				HDdouble torque[3] = {0.0, 0.0, 0.0};
				hdBeginFrame(hHDm);
				hdGetDoublev(HD_CURRENT_JOINT_ANGLES, p_state->position);

				constexpr double angle_final_effector = 15.0 * pi / 180.0;
				const double a3 = sqrt(pow(0.135, 2) + pow(0.04, 2) - 2.0 * 0.135 * 0.04 * cos(pi - angle_final_effector));
				const double gamma_final_effector = asin(0.04 * sin(pi - angle_final_effector) / a3);

				torque[0] = -1000.0 * taum[0];
				torque[1] = 1000.0 * taum[1];
				torque[2] = 1000.0 * taum[2];

				qm[0] = -state.position[0];
				qm[1] = state.position[1];
				qm[2] = state.position[2] - 0.5 * pi - qm[1] - gamma_final_effector;

				hdSetDoublev(HD_CURRENT_JOINT_TORQUE, torque);

				hdEndFrame(hHDm);
				return HD_CALLBACK_CONTINUE;
}

// Creates the connection to the robot, can be used to create a connection to a slave robot
// TODO: work in the slave robot connection
void CHapticTemplateDlg::on_bn_clicked_initialize()
{
				HDErrorInfo error;
    const HDstring master_robot = "Default Device";
				hHDm = hdInitDevice(master_robot);
				if (HD_DEVICE_ERROR(error = hdGetError()))
				{
								MessageBox(_T("Master Device not Found!"));
								return;
				}

				// call to servo_loop_callback which handles movement according to a given taum and torque
				servo_loop_handle = hdScheduleAsynchronous(ServoLoopCallback, &state, HD_MAX_SCHEDULER_PRIORITY);

				hdMakeCurrentDevice(hHDm);
				if (!hdIsEnabled(HD_FORCE_OUTPUT))
								hdEnable(HD_FORCE_OUTPUT);

				if (HD_DEVICE_ERROR(error = hdGetError()))
								MessageBox(_T("Force output enable error!"));

				if (!scheduler_started)
				{
								hdStartScheduler();
								scheduler_started = true;
								Sleep(1500);
				}


				if (HD_DEVICE_ERROR(error = hdGetError()))
				{
								MessageBox(_T("Servo loop initialization error"));

								hdDisableDevice(hHDm);
								exit(-1);
				}
				else
				{
								initialized = true;
								m_statusTextBox.SetWindowTextW(_T("*** Phantom Robot initialized ***"));
				}

				timeBeginPeriod(1);
}

// Assigns the calibration parameters established in the driver to the robot
void CHapticTemplateDlg::on_bn_clicked_calibration()
{
				if (initialized)
				{
								int supported_calibration_styles;
								int calibrationStyle;
								HDErrorInfo error;

								hdGetIntegerv(HD_CALIBRATION_STYLE, &supported_calibration_styles);

								if (supported_calibration_styles & HD_CALIBRATION_INKWELL)
												calibrationStyle = HD_CALIBRATION_INKWELL;
								else
								{
												MessageBox(_T(" Sorry, no ink-well calibration available "));
												return;
								}

								if (HD_DEVICE_ERROR(error = hdGetError()))
												m_statusTextBox.SetWindowTextW(_T("*** Failed to start the scheduler ***"));

								HDenum status;
								hdScheduleSynchronous(CalibrationStatusCallback, &status, HD_DEFAULT_SCHEDULER_PRIORITY);

								if (status == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
												MessageBox(_T(" Please put the device into the ink-well "));
								else
								{
												m_statusTextBox.SetWindowTextW(_T("*** Calibration done ***"));
								}
				}
				else
				{
								MessageBox(_T(" Please initialize first the Phantom device "));
				}
}

// Reads the encoder values and displays it
// TODO: make it an async function so that there's no need to keep clicking
// the button
void CHapticTemplateDlg::on_bn_clicked_read() {
				CString text[3];
				text[0].Format(L"%.3f", qm[0] * 180.0 / pi);
				text[1].Format(L"%.3f", qm[1] * 180.0 / pi);
				text[2].Format(L"%.3f", qm[2] * 180.0 / pi);
				m_Encoder1.SetWindowTextW(text[0]);
				m_Encoder2.SetWindowTextW(text[1]);
				m_Encoder3.SetWindowTextW(text[2]);

				return;
}

// Returns to the home position for the robot, uses a small PID code to do so
// Can be adjusted to return faster to home but, so far the convergence is right
void CALLBACK CHapticTemplateDlg::home_timer_proc(UINT uId, UINT u_msg, DWORD_PTR dw_user, DWORD_PTR dw1, DWORD_PTR dw2) {
				static double ti = 0.0, tf = 2.0;
				static double bm0[no_joints] = { 0.0 }, bm3[no_joints] = { 0.0 }, bm4[no_joints] = { 0.0 }, bm5[no_joints] = { 0.0 };
				static double em_1[no_joints] = { 0.0 };

				constexpr double qmdf[no_joints] = { 0.0, 90 * pi / 180, -90 * pi / 180 };
				constexpr double kpm[no_joints] = { 1.2, 1.2, 1.2 };
				constexpr double kim[no_joints] = { 0.2, 0.2, 0.2 };
				constexpr double kdm[no_joints] = { 0.1, 0.1, 0.1 };

				double qmd[no_joints] = { 0.0 }, qpmd[no_joints] = {0.0};
				double em[no_joints] = {0.0}, emp[no_joints] = {0.0}, emi[no_joints] = {0.0};
				double t = 0.0;

				CString time;

				auto pMainWnd = dynamic_cast<CHapticTemplateDlg*>(AfxGetApp()->m_pMainWnd);
				
				if (i_c_home) {
								ti = timeGetTime();

								for (int i = 0; i < no_joints; i++) {
												bm0[i] = qm[i];
												bm3[i] = 10.0 * (qmdf[i] - qm[i]) / pow(tf, 3);
												bm4[i] = -15.0 * (qmdf[i] - qm[i]) / pow(tf, 4);
												bm5[i] = 6.0 * (qmdf[i] - qm[i]) / pow(tf, 5);
								}

								pMainWnd->m_statusTextBox.SetWindowTextW(_T("*** Reaching home position ***"));
				}

				t = (timeGetTime() - ti) / 1000.0;
				if (t <= tf) {
								time.Format(_T("%f"), t);
								pMainWnd->m_time.SetWindowTextW(time);
				}
				else {
								t = tf;
								time.Format(_T("%f"), t);
								pMainWnd->m_time.SetWindowTextW(time);
				}

				if (ti <= 0.001) {
								std::copy(std::begin(bm0), std::end(bm0), std::begin(qm));
				}

				for (int i = 0; i < no_joints; i++) {
								if (t <= tf) {
												qmd[i] = bm5[i]*pow(t, 5) + bm4[i]*pow(t, 4) + bm3[i]*pow(t, 3) + bm0[i];
												qpmd[i] = 5.0*bm5[i]*pow(t, 4) + 4.0*bm4[i]*pow(t, 3) + 3.0*bm3[i]*pow(t, 2);
								}
								else {
												qmd[i] = qmdf[i];
								}
				}

				for (int i = 0; i < no_joints; i++) {
								em[i] = qm[i] - qmd[i];
								emi[i] += em[i] * sample_time;
								em_1[i] = em[i];
								emp[i] = (em[i] - em_1[i]) / sample_time;

								taum[i] = -kpm[i] * em[i] - kim[i] * emi[i] - kdm[i] * emp[i];
				}

				if (t > tf && completed) {
								pMainWnd->m_statusTextBox.SetWindowTextW(_T("*** Home position ***"));
								completed = false;
				}

				if (i_c_home) i_c_home = false;
}

// Calls the above function with a timer to reach home 
// position in a certain time
void CHapticTemplateDlg::on_bn_clicked_home() {
				if (!i_c_home) {
								timeKillEvent(home_timer_id);
								home_completed_flag = i_c_home = true;
				}
				if (!i_c_smc) {
								timeKillEvent(smc_timer_id);
				}

				home_timer_id = timeSetEvent(static_cast<UINT>(sample_time * 1000.0), 0, home_timer_proc, 0, TIME_PERIODIC); //Home timer initialization
}

void CHapticTemplateDlg::write_data_to_file(std::vector<double>& graph_data)
{
    for(const auto d: graph_data)
    {
								graph_file << d << ",";
    }

    graph_file << "\n";
}

// Main function for movement, calculates the different components for the given control, despite parra-vega controller being
// added, it doesn't work, I'll add it later
// 26-08-24 parra-vega should work now, made some adjustments
// 30-08-24 while they calculate values correctly, still have to figure out a way to make the work
void CALLBACK CHapticTemplateDlg::smc_timer_proc(UINT u_id, UINT u_msg, DWORD_PTR dw_user, DWORD_PTR dw1, DWORD_PTR dw2) {
				auto pMainWnd = dynamic_cast<CHapticTemplateDlg*>(AfxGetApp()->m_pMainWnd);
				CString time;

				static double ti = 0.0;

				if (i_c_smc) {
								ti = timeGetTime();
								pMainWnd->m_statusTextBox.SetWindowTextW(_T("*** Control in progress ***"));
				}

				time.Format(_T("%f"), (timeGetTime() - ti) / 1000.0);
				pMainWnd->m_time.SetWindowTextW(time);

				std::vector<std::vector<double>> aux = controllers::pid_controller(pi, sample_time, i_c_smc, qm, ti);
				//std::vector<double> aux = Controllers::ParraVegaController(PI, SAMPLE_TIME, iCSmc, ti);
				//std::vector<double> aux = Controllers::NLController(PI, SAMPLE_TIME, pMainWnd, qm, ti);

				std::copy(aux[0].begin(), aux[0].end(), taum.begin());
				write_data_to_file(aux[1]);
				aux.swap(std::vector<std::vector<double>>());

				if (i_c_smc) i_c_smc = false;
}

void CHapticTemplateDlg::on_bn_clicked_smc() {
				if (!i_c_home) timeKillEvent(home_timer_id);

				if (!i_c_smc) {
								timeKillEvent(smc_timer_id);
								smc_completed_flag = i_c_smc = true;
				}

				smc_timer_id = timeSetEvent(static_cast<UINT>(sample_time * 1000.0), 0, smc_timer_proc, 0, TIME_PERIODIC);
}