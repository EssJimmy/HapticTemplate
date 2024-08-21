
// HapticTemplateDlg.cpp : implementation file
//

#include "pch.h"
#include "HapticTemplateDlg.h"
#include "afxdialogex.h"
#include <mmsystem.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif // DEBUG 

typedef struct {
				hduVector3Dd position;
} DeviceStateStruct;


const int MAX_GRAF_ROWS = 4000;
const int grafSkip = 0;
const int NO_JOINTS = 3;
const double SAMPLE_TIME = 0.001;
const double PI = 3.1415926535;

int index = 0;
double qm[NO_JOINTS] = { 0.0 };
double taum[NO_JOINTS] = { 0.0 };
double grafi[MAX_GRAF_ROWS][25] = { 0.0 };
bool initialized = false, schedulerStarted = false;

HHD hHDm;
HDSchedulerHandle servoLoopHandle;
DeviceStateStruct state;

//Timers
MMRESULT HomeTimerID;
MMRESULT SmcTimerID;
//bool iCTele=true, PID= true;
//Banderas
bool iCHome = true, Home = true, homeCompletedFlag = true;
bool iCSmc = true, Smc = true, SmcCompletedFlag = true;
//Bandera para matar toos los timers
bool CompletedFlag = true;

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
				if (!Smc || !Home) {
								timeKillEvent(SmcTimerID);
								CompletedFlag = Smc = Home = true;
				}

				if (initialized && hdIsEnabled(HD_FORCE_OUTPUT))
								hdDisable(HD_FORCE_OUTPUT);

				hdUnschedule(servoLoopHandle);
				if (schedulerStarted) hdStopScheduler();

				if (initialized) hdDisableDevice(hHDm);

				FILE* outFile;
				if (fopen_s(&outFile, "DATOS.m", "w") != 0)
								MessageBox(_T("No se pudo crear el archivo para graficar"));
				else {
								for (int i = 0; i < index; i++) {
												for (int j = 0; j < 25; j++) {
																fprintf(outFile, "%f \t", grafi[i][j]);
												}

												fprintf(outFile, "\n");
								}

								fclose(outFile);
				}
				exit(0);
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CHapticTemplateDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

static HDCallbackCode HDCALLBACK CalibrationStatusCallback(void* pUserData)
{
				HDenum* pStatus = (HDenum*)pUserData;

				hdBeginFrame(hHDm);
				hdUpdateCalibration(HD_CALIBRATION_INKWELL);
				*pStatus = hdCheckCalibration();
				hdEndFrame(hHDm);

				return HD_CALLBACK_DONE;
}

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


void CHapticTemplateDlg::OnBnClickedInitialize()
{
				// TODO: Add your control notification handler code here
				HDErrorInfo error;
				HDstring MasterRobot = "Default Device";
				//HDstring SlaveRobot = "p2";
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

void CHapticTemplateDlg::OnBnClickedCalib()
{
				// TODO: Add your control notification handler code here
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

void CALLBACK CHapticTemplateDlg::HomeTimerProc(UINT uId, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2) {
				static int grafFlag = 0;
				static double ti = 0.0, tf = 2.0;
				double t;

				double bm0[NO_JOINTS] = { 0.0 }, bm3[NO_JOINTS] = { 0.0 }, bm4[NO_JOINTS] = { 0.0 }, bm5[NO_JOINTS] = { 0.0 };

				const double qmdf[NO_JOINTS] = { 0.0, 90 * PI / 180, -90 * PI / 180 };
				double qmd[NO_JOINTS] = { 0.0 }, qpmd[3] = { 0.0 };

				double em_1[NO_JOINTS] = { 0.0 }, em[NO_JOINTS] = { 0.0 }, emp[NO_JOINTS] = { 0.0 }, emi[NO_JOINTS] = { 0.0 };

				const double kpm[NO_JOINTS] = { 1.2, 1.2, 1.2 };
				const double kim[NO_JOINTS] = { 0.2, 0.2, 0.2 };
				const double kdm[NO_JOINTS] = { 0.1, 0.1, 0.1 };

				CHapticTemplateDlg* pMainWnd = (CHapticTemplateDlg*)AfxGetApp()->m_pMainWnd;
				CString time;
				if (iCHome) {
								ti = timeGetTime();

								for (int i = 0; i < NO_JOINTS; i++) {
												bm0[i] = qm[i];
												bm3[i] = 10 * (qmdf[i] - qm[i]) / pow(tf, 3);
												bm4[i] = -15 * (qmdf[i] - qm[i]) / pow(tf, 4);
												bm5[i] = 6 * (qmdf[i] - qm[i]) / pow(tf, 5);
								}

								pMainWnd->m_statusTextBox.SetWindowTextW(_T("*** Reaching home position ***"));
				}

				t = (timeGetTime() - ti) / 1000.0;
				if (t <= tf) {
								time.Format(_T("%f"), t);
								pMainWnd->m_time.SetWindowTextW(time);
				}

				if (ti <= 0.001) {
								std::copy(std::begin(bm0), std::end(bm0), std::begin(qm));
				}

				for (int i = 0; i < NO_JOINTS; i++) {
								if (t <= tf) {
												qmd[i] = bm5[i] * pow(t, 5) + bm4[i] * pow(t, 4) + bm3[i] * pow(t, 3) + bm0[i];
												qpmd[i] = 5 * bm5[i] * pow(t, 4) + 4 * bm4[i] * pow(t, 3) + 3 * bm3[i] * pow(t, 2);
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

				if (t > tf && CompletedFlag) {
								pMainWnd->m_statusTextBox.SetWindowTextW(_T("*** Home position ***"));
								CompletedFlag = false;
				}

				if (iCHome) iCHome = false;

				return;
}

void CHapticTemplateDlg::OnBnClickedHome() {
				if (!iCHome) {
								timeKillEvent(HomeTimerID);
								homeCompletedFlag = iCHome = true;
				}
				if (!iCSmc) {
								timeKillEvent(SmcTimerID);
				}

				HomeTimerID = timeSetEvent(SAMPLE_TIME * 1000, 0, HomeTimerProc, 0, TIME_PERIODIC); //Home timer initialization
}

void CALLBACK CHapticTemplateDlg::SmcTimerProc(UINT uID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2) {
				CString TIEMPO;
				static int index = 0;

				static int grafFlag = 0;
				static double ti = 0;
				static double qm_1[NO_JOINTS] = { 0, 90.0 * PI / 180.0, -90.0 * PI / 180.0 };
				static double sigma[NO_JOINTS] = { 0.0,0.0,0.0 };
				const double alpha = 9.0 / 11.0;

				const double qmdf[NO_JOINTS] = { 0, 90.0 * PI / 180.0, -90.0 * PI / 180.0 };
				const double qmd[NO_JOINTS] = { 0, 0, 0 };
				const double lambdaZero[NO_JOINTS] = { 2, 2, 2 };
				const double lambdaOne[NO_JOINTS] = { 1.5, 1.5, 1.5 };
				const double lambdaTwo[NO_JOINTS] = { 1.1, 1.1, 1.1 };

				double dqm[NO_JOINTS] = { 0, 0, 0 };
				double t;
				double qd1 = 0;
				double qd2 = 0;
				double qd3 = 0;
				double dqd1 = 0;
				double dqd2 = 0;
				double dqd3 = 0;

				//ganancias PId
				const double kp[NO_JOINTS] = { 1.5,3.5,2.8 };	//Ganancias Proporcional
				const double ki[NO_JOINTS] = { 0.2,0.25,0.25 }; 	//Ganancias Integral   
				const double kd[NO_JOINTS] = { 0.1,0.2,0.2 };

				double dotePos[NO_JOINTS] = { 0.0,0.0,0.0 };
				double ePos[NO_JOINTS] = { 0.0,0.0,0.0 };
				double sq[NO_JOINTS] = { 0.0,0.0,0.0 };
				double dotqr[NO_JOINTS] = { 0.0,0.0,0.0 };
				double s[NO_JOINTS] = { 0.0,0.0,0.0 };
				double qd[NO_JOINTS] = { 0.0,0.0,0.0 };
				double dqd[NO_JOINTS] = { 0.0,0.0,0.0 };
				double dotsigma[NO_JOINTS] = { 0.0,0.0,0.0 };
				double ePosi[NO_JOINTS] = { 0.0, 0.0, 0.0 };
				double vel[NO_JOINTS] = { 0.0, 0.0, 0.0 };

				//ganancias parra-vega
				const double gammaPV[NO_JOINTS] = { 0.4, 0.4, 0.4 };
				const double alphaPV[NO_JOINTS] = { 14.0, 14.0, 14.0 };
				const double kdPV[NO_JOINTS] = { 0.35, 0.35, 0.35 };

				static double sigmaPV[NO_JOINTS] = { 0.0, 0.0, 0.0 };
				double dotqrPV[NO_JOINTS] = { 0.0, 0.0, 0.0 };
				double dotsigmaPV[NO_JOINTS] = { 0.0, 0.0, 0.0 };
				double sPV[NO_JOINTS] = { 0.0, 0.0, 0.0 };
				double sdPV[NO_JOINTS] = { 0.0, 0.0, 0.0 };
				double srPV[NO_JOINTS] = { 0.0, 0.0, 0.0 };
				double sqPV[NO_JOINTS] = { 0.0, 0.0, 0.0 };

				const double K1[NO_JOINTS] = { 11, 16, 12 };
				const double K2[NO_JOINTS] = { 0.05, 0.08, 0.06 };
				const double K3[NO_JOINTS] = { 0.05, 0.05, 0.05 };
				const double K4[NO_JOINTS] = { 0.16, 0.28, 0.26 };
				const double KV[NO_JOINTS] = { 0.08, 0.08, 0.08 };

				CHapticTemplateDlg* pMainWnd = (CHapticTemplateDlg*)AfxGetApp()->m_pMainWnd;

				if (iCSmc) {
								ti = timeGetTime();
								pMainWnd->m_statusTextBox.SetWindowTextW(_T("*** Control in progress ***"));

								std::copy(std::begin(qm), std::end(qm), std::begin(qm_1));
				}

				t = (timeGetTime() - ti) / 1000.0;
				TIEMPO.Format(_T("%f"), t);
				pMainWnd->m_time.SetWindowTextW(TIEMPO);

				vel[0] = HelperFunctions::RED1(qm[0], SAMPLE_TIME);
				vel[1] = HelperFunctions::RED2(qm[1], SAMPLE_TIME);
				vel[2] = HelperFunctions::RED3(qm[2], SAMPLE_TIME);

				qd[0] = -0.4 + 0.4 * cos(3 * t);
				qd[1] = 1.37 + 0.2 * cos(t);
				qd[2] = -1.37 - 0.2 * cos(2 * t);

				dqd[0] = -0.12 * sin(3 * t);
				dqd[1] = -0.2 * sin(t);
				dqd[2] = 0.4 * sin(2 * t);

				for (int i = 0; i < NO_JOINTS; i++) {
								//PID normal
								ePos[i] = qm[i] - qd[i];
								dotePos[i] = vel[i] - dqd[i];
								sq[i] = dotePos[i] + K1[i] * HelperFunctions::Sign(ePos[i]) * (pow((abs(ePos[i])), alpha));
								dotqr[i] = dqd[i] - K1[i] * HelperFunctions::Sign(ePos[i]) * pow((abs(ePos[i])), alpha) - K2[i] * sigma[i];
								s[i] = dqm[i] - dotqr[i];
								dotsigma[i] = K3[i] * sq[i] + HelperFunctions::Sign(sq[i]);
								sigma[i] += dotsigma[i] * SAMPLE_TIME;
								ePosi[i] += ePos[i] * SAMPLE_TIME;

								taum[i] = -kp[i] * ePos[i] - ki[i] * ePosi[i] - kd[i] * dotePos[i];

								//parra-vega
								sigmaPV[i] += dotsigmaPV[i] * SAMPLE_TIME;
								dotqrPV[i] = dqd[i] - alphaPV[i] * ePos[i] + sdPV[i] - gammaPV[i] * sigmaPV[i];
								dotsigmaPV[i] = HelperFunctions::Sign(sqPV[i]);
								sPV[i] = dotePos[i] + alphaPV[i] * ePos[i];
								sqPV[i] = sPV[i];
								srPV[i] = sqPV[i] + gammaPV[i] * sigmaPV[i];
				}

				if (iCSmc) {
								index = 0;
								iCSmc = false;
				}

				if (!grafFlag) {
								grafi[index][0] = t;
								grafi[index][1] = qm[0] * 180.0 / PI;
								grafi[index][2] = qm[1] * 180.0 / PI;
								grafi[index][3] = qm[2] * 180.0 / PI;
								grafi[index][4] = dotePos[0] * 180.0 / PI;
								grafi[index][5] = dotePos[1] * 180.0 / PI;
								grafi[index][6] = dotePos[2] * 180.0 / PI;
								grafi[index][7] = ePos[0] * 180.0 / PI;
								grafi[index][8] = ePos[1] * 180.0 / PI;
								grafi[index][9] = ePos[2] * 180.0 / PI;
								grafi[index][10] = qd[0] * 180.0 / PI;
								grafi[index][11] = qd[1] * 180.0 / PI;
								grafi[index][12] = qd[2] * 180.0 / PI;
								grafi[index][13] = abs(taum[0]);
								grafi[index][14] = abs(taum[1]);
								grafi[index][15] = abs(taum[2]);
								grafi[index][16] = dqd[0] * 180.0 / PI;
								grafi[index][17] = dqd[1] * 180.0 / PI;
								grafi[index][18] = dqd[2] * 180.0 / PI;
								grafi[index][19] = vel[0] * 180.0 / PI;
								grafi[index][20] = vel[1] * 180.0 / PI;
								grafi[index][21] = vel[2] * 180.0 / PI;
								grafi[index][22] = dotqr[0];
								grafi[index][23] = dotqr[1];
								grafi[index][24] = dotqr[2];

								index++;
								grafFlag = grafSkip + 1;
				}

				grafFlag--;

				return;
}

void CHapticTemplateDlg::OnBnClickedSmc() {
				if (!iCHome) timeKillEvent(HomeTimerID);

				if (!iCSmc) {
								timeKillEvent(SmcTimerID);
								SmcCompletedFlag = iCSmc = true;
				}

				SmcTimerID = timeSetEvent(SAMPLE_TIME * 1000, 0, SmcTimerProc, 0, TIME_PERIODIC);
}

/*
double CHapticTemplateDlg::Sign(double f) {
				double s = 0;
				if (f > 0) s = 1;
				else if (f < 0) s = -1;

				return s;
}

double CHapticTemplateDlg::RED1(double posq1) {
				static double u = 0, u1 = 0, dx = 0, x = 0, du = 0;
				double aux1 = abs(x - posq1), aux2 = 0.5, s = Sign(x - posq1);

				u1 += du * SAMPLE_TIME;
				x += dx * SAMPLE_TIME;
				u = u1 - 5.5 * (pow(aux1, aux2)) * s;
				dx = u;
				du = -0.7 * s;

				return u;
}

double CHapticTemplateDlg::RED2(double posq2) {
				static double u = 0, u1 = 0, x = 1.57;
				double dx = u, s = Sign(x - posq2);

				u = u1 - 1.5 * (pow(abs(x - posq2), 0.5)) * s;
				u1 += (-0.8 * s) * SAMPLE_TIME;
				x += dx * SAMPLE_TIME;

				return u;
}

double CHapticTemplateDlg::RED3(double posq3) {
				static double u = 0, u1 = 0, x = -1.57;
				double dx = u, s = Sign(x - posq3);

				u = u1 - 1.5 * (pow(abs(x - posq3), 0.5)) * s;
				u1 += (-0.8 * s) * SAMPLE_TIME;
				x += dx * SAMPLE_TIME;

				return u;
}

double CHapticTemplateDlg::Levantq1(double pos1) {
				static double z0 = 0.0;
				static double z1 = 0.0;
				static double z2 = 0.0;

				const double L = 3;

				double v0, v1, dz2;

				v0 = -(1.1) * (pow(L, 0.33)) * (pow(abs(z0 - pos1), 0.66)) * (Sign(z0 - pos1)) + z1;
				v1 = -(1.5) * (pow(L, 0.5)) * (pow(abs(z1 - pos1), 0.5)) * (Sign(z1 - v0)) + z2;
				dz2 = -2.0 * L * (Sign(z2 - v1));

				z0 += v0 * SAMPLE_TIME;
				z1 += v1 * SAMPLE_TIME;
				z2 += dz2 * SAMPLE_TIME;

				return z1;
}

double CHapticTemplateDlg::Levantq2(double pos2) {
				const double L = 3.0;

				static double z0 = 1.65;
				static double z1 = 0.0;
				static double z2 = 0.0;

				double v0, v1, dz2;

				v0 = -(1.1) * (pow(L, 1 / 3)) * (pow((abs(z0 - pos2)), (2 / 3))) * (Sign(z0 - pos2)) + z1;
				v1 = -(1.5) * (pow(L, 1 / 2)) * (pow((abs(z1 - pos2)), (1 / 2))) * (Sign(z1 - v0)) + z2;
				dz2 = -(2.0) * L * (Sign(z2 - v1));

				z0 += v0 * SAMPLE_TIME;
				z1 += v1 * SAMPLE_TIME;
				z2 += dz2 * SAMPLE_TIME;

				return z1;
}

double CHapticTemplateDlg::Levantq3(double pos3) {
				const double L = 3.0;

				static double z0 = -1.74;
				static double z1 = 0.0;
				static double z2 = 0.0;

				double v0, v1, dz2;

				v0 = -(1.1) * (pow(L, 1 / 3)) * (pow((abs(z0 - pos3)), (2 / 3))) * (Sign(z0 - pos3)) + z1;
				v1 = -(1.5) * (pow(L, 1 / 2)) * (pow((abs(z1 - pos3)), (1 / 2))) * (Sign(z1 - v0)) + z2;
				dz2 = -(2.0) * L * (Sign(z2 - v1));

				z0 += v0 * SAMPLE_TIME;
				z1 += v1 * SAMPLE_TIME;
				z2 += dz2 * SAMPLE_TIME;

				return z1;
}
*/