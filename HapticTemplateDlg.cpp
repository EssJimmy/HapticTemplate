// HapticTemplateDlg.cpp : implementation file
// TODO: revise trajectory graphing so that it works and no longer throws memory exceptions
#include "HapticTemplateDlg.h"
#include "HelperFunctions.h"
#include "Controllers.h"
#include "afxdialogex.h"
#include <mmsystem.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif // DEBUG 

// Value inside needs to store diferent values
// but struct itself needs to be a pointer, thus needing
// this weird arangement
typedef struct {
				hduVector3Dd position;
} DeviceStateStruct;

// constant values for calculations and other stuff
const int MAX_GRAF_ROWS = 4000;
const int NO_JOINTS = 3;
const double SAMPLE_TIME = 0.001;
const double PI = 3.1415926535;

// non constant global values needed in several functions
double qm[NO_JOINTS] = { 0.0 };
std::vector<double> taum(NO_JOINTS);

// haptic robot stuff
HHD hHDm;
HDSchedulerHandle servoLoopHandle;
DeviceStateStruct state;

//Timers
MMRESULT HomeTimerID;
MMRESULT SmcTimerID;

//flags
bool initialized = false, schedulerStarted = false;
bool iCHome = true, homeFlag = true, homeCompletedFlag = true;
bool iCSmc = true, smcFlag = true, smcCompletedFlag = true;
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
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CHapticTemplateDlg dialog
CHapticTemplateDlg::CHapticTemplateDlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_HAPTICTEMPLATE_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CHapticTemplateDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_ENCODER4, m_time);
	DDX_Control(pDX, IDC_ENCODER1, m_Encoder1);
	DDX_Control(pDX, IDC_ENCODER5, m_statusTextBox);
	DDX_Control(pDX, IDC_ENCODER2, m_Encoder2);
	DDX_Control(pDX, IDC_ENCODER3, m_Encoder3);
}

BEGIN_MESSAGE_MAP(CHapticTemplateDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
				ON_BN_CLICKED(IDC_INITIALIZE, &CHapticTemplateDlg::OnBnClickedInitialize)
				ON_BN_CLICKED(IDC_CALIB, &CHapticTemplateDlg::OnBnClickedCalib)
				ON_BN_CLICKED(IDC_REED, &CHapticTemplateDlg::OnBnClickedReed)
				ON_BN_CLICKED(IDC_PID, &CHapticTemplateDlg::OnBnClickedHome)
				ON_BN_CLICKED(IDC_SMC, &CHapticTemplateDlg::OnBnClickedSmc)
END_MESSAGE_MAP()


// CHapticTemplateDlg message handlers

BOOL CHapticTemplateDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != nullptr)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CHapticTemplateDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
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
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

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
				if (!smcFlag || !homeFlag) {
								timeKillEvent(SmcTimerID);
								completed = smcFlag = homeFlag = true;
				}

				if (initialized && hdIsEnabled(HD_FORCE_OUTPUT))
								hdDisable(HD_FORCE_OUTPUT);

				hdUnschedule(servoLoopHandle);
				if (schedulerStarted) hdStopScheduler();

				if (initialized) hdDisableDevice(hHDm);

				exit(0);
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CHapticTemplateDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


// Callback to calibration parameters stablished by the drivers
static HDCallbackCode HDCALLBACK CalibrationStatusCallback(void* pUserData)
{
				HDenum* pStatus = (HDenum*)pUserData;

				hdBeginFrame(hHDm);
				hdUpdateCalibration(HD_CALIBRATION_INKWELL);
				*pStatus = hdCheckCalibration();
				hdEndFrame(hHDm);

				return HD_CALLBACK_DONE;
}

// How the robot moves, by using a standard defined torque and the taum calculated in the
// SmcTimerProc method
static HDCallbackCode HDCALLBACK ServoLoopCallback(void* pUserData) {
				DeviceStateStruct* pState = static_cast<DeviceStateStruct*>(pUserData);
				HDdouble torque[3] = {0.0, 0.0, 0.0};
				hdBeginFrame(hHDm);
				hdGetDoublev(HD_CURRENT_JOINT_ANGLES, pState->position);

				const double angleFinalEffector = 15.0 * PI / 180.0;
				const double a3 = sqrt(pow(0.135, 2) + pow(0.04, 2) - 2.0 * 0.135 * 0.04 * cos(PI - angleFinalEffector));
				const double gammaFinalEffector = asin(0.04 * sin(PI - angleFinalEffector) / a3);

				torque[0] = -1000.0 * taum[0];
				torque[1] = 1000.0 * taum[1];
				torque[2] = 1000.0 * taum[2];

				qm[0] = -state.position[0];
				qm[1] = state.position[1];
				qm[2] = state.position[2] - 0.5 * PI - qm[1] - gammaFinalEffector;

				hdSetDoublev(HD_CURRENT_JOINT_TORQUE, torque);

				hdEndFrame(hHDm);
				return HD_CALLBACK_CONTINUE;
}

// Creates the connection to the robot, can be used to create a connection to a slave robot
// TODO: work in the slave robot connecction
void CHapticTemplateDlg::OnBnClickedInitialize()
{
				HDErrorInfo error;
				HDstring MasterRobot = "Default Device";
				hHDm = hdInitDevice(MasterRobot);
				if (HD_DEVICE_ERROR(error = hdGetError()))
				{
								MessageBox(_T("Master Device not Found!"));
								return;
				}

				servoLoopHandle = hdScheduleAsynchronous(ServoLoopCallback, &state, HD_MAX_SCHEDULER_PRIORITY);

				hdMakeCurrentDevice(hHDm);
				if (!hdIsEnabled(HD_FORCE_OUTPUT))
								hdEnable(HD_FORCE_OUTPUT);

				if (HD_DEVICE_ERROR(error = hdGetError()))
								MessageBox(_T("Force output enable error!"));

				if (!schedulerStarted)
				{
								hdStartScheduler();
								schedulerStarted = true;
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

// Assigns the calibration parameters stablished in the driver to the robot
void CHapticTemplateDlg::OnBnClickedCalib()
{
				if (initialized)
				{
								int supportedCalibrationStyles;
								int calibrationStyle;
								HDErrorInfo error;

								hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);

								if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
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

								return;
				}
				else
				{
								MessageBox(_T(" Please initialize first the Phantom device "));
				}
}

// Reads the encoder values and displays it
// TODO: make it an async function so that there's no need to keep clicking
// the button
void CHapticTemplateDlg::OnBnClickedReed() {
				CString text[3];
				text[0].Format(L"%.3f", qm[0] * 180.0 / PI);
				text[1].Format(L"%.3f", qm[1] * 180.0 / PI);
				text[2].Format(L"%.3f", qm[2] * 180.0 / PI);
				m_Encoder1.SetWindowTextW(text[0]);
				m_Encoder2.SetWindowTextW(text[1]);
				m_Encoder3.SetWindowTextW(text[2]);

				return;
}

// Returns to the home position for the robot, uses a small PID code to do so
// Can be adjusted to return faster to home but, so far the convergence is right
void CALLBACK CHapticTemplateDlg::HomeTimerProc(UINT uId, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2) {
				static int grafFlag = 0;
				static double ti = 0.0, tf = 2.0;
				static double bm0[NO_JOINTS] = { 0.0 }, bm3[NO_JOINTS] = { 0.0 }, bm4[NO_JOINTS] = { 0.0 }, bm5[NO_JOINTS] = { 0.0 };
				static double em_1[NO_JOINTS] = { 0.0 };

				const double qmdf[NO_JOINTS] = { 0.0, 90 * PI / 180, -90 * PI / 180 };
				const double kpm[NO_JOINTS] = { 1.2, 1.2, 1.2 };
				const double kim[NO_JOINTS] = { 0.2, 0.2, 0.2 };
				const double kdm[NO_JOINTS] = { 0.1, 0.1, 0.1 };

				double qmd[NO_JOINTS] = { 0.0 }, qpmd[3] = { 0.0 };
				double em[NO_JOINTS] = {0.0}, emp[NO_JOINTS] = {0.0}, emi[NO_JOINTS] = {0.0};
				double t = 0.0;

				CString time;

				CHapticTemplateDlg* pMainWnd = (CHapticTemplateDlg*)AfxGetApp()->m_pMainWnd;
				
				if (iCHome) {
								ti = timeGetTime();

								for (int i = 0; i < NO_JOINTS; i++) {
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

				for (int i = 0; i < NO_JOINTS; i++) {
								if (t <= tf) {
												qmd[i] = bm5[i]*pow(t, 5) + bm4[i]*pow(t, 4) + bm3[i]*pow(t, 3) + bm0[i];
												qpmd[i] = 5.0*bm5[i]*pow(t, 4) + 4.0*bm4[i]*pow(t, 3) + 3.0*bm3[i]*pow(t, 2);
								}
								else {
												qmd[i] = qmdf[i];
								}
				}

				for (int i = 0; i < NO_JOINTS; i++) {
								em[i] = qm[i] - qmd[i];
								emi[i] += em[i] * SAMPLE_TIME;
								em_1[i] = em[i];
								emp[i] = (em[i] - em_1[i]) / SAMPLE_TIME;

								taum[i] = -kpm[i] * em[i] - kim[i] * emi[i] - kdm[i] * emp[i];
				}

				if (t > tf && completed) {
								pMainWnd->m_statusTextBox.SetWindowTextW(_T("*** Home position ***"));
								completed = false;
				}

				if (iCHome) iCHome = false;

				// freeing memory back to the system
				std::free(qmd); std::free(qpmd); std::free(em); std::free(emi);

				return;
}

// Calls the above function with a timer to reach home 
// position in a certain time
void CHapticTemplateDlg::OnBnClickedHome() {
				if (!iCHome) {
								timeKillEvent(HomeTimerID);
								homeCompletedFlag = iCHome = true;
				}
				if (!iCSmc) {
								timeKillEvent(SmcTimerID);
				}

				HomeTimerID = timeSetEvent(SAMPLE_TIME * 1000, 0, HomeTimerProc,(DWORD) 0, TIME_PERIODIC); //Home timer initialization
}

// Main function for movement, calculates the different components for the given control, despite parra-vega controller being
// added, it doesnt work, i'll add it later
void CALLBACK CHapticTemplateDlg::SmcTimerProc(UINT uID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2) {
				CHapticTemplateDlg* pMainWnd = (CHapticTemplateDlg*)AfxGetApp()->m_pMainWnd;

				taum = Controllers::PIDController(PI, SAMPLE_TIME, qm, pMainWnd, iCSmc);
				//taum = Controllers::ParraVegaController(PI, SAMPLE_TIME, qm, pMainWnd, iCSmc);

				return;
}

void CHapticTemplateDlg::OnBnClickedSmc() {
				if (!iCHome) timeKillEvent(HomeTimerID);

				if (!iCSmc) {
								timeKillEvent(SmcTimerID);
								smcCompletedFlag = iCSmc = true;
				}

				SmcTimerID = timeSetEvent(SAMPLE_TIME * 1000, 0, SmcTimerProc, 0, TIME_PERIODIC);
}