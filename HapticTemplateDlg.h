
// HapticTemplateDlg.h : header file
//

#pragma once
#include "pch.h"
#include "framework.h"
#include "stdafx.h"
#include "HapticTemplate.h"
#include <vector>
#include <afxwin.h>
#include <HDU/hduVector.h>
#include <GL/glut.h>

// CHapticTemplateDlg dialog
class CHapticTemplateDlg : public CDialogEx
{
// Construction
public:
	CHapticTemplateDlg(CWnd* p_parent = nullptr);	// standard constructor

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_HAPTICTEMPLATE_DIALOG };
#endif

	protected:
void DoDataExchange(CDataExchange* p_dx) override;	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT n_id, LPARAM l_param);
	afx_msg void OnPaint();
	afx_msg void OnClose();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()

public:
    static void CALLBACK home_timer_proc(UINT u_id, UINT u_msg, DWORD_PTR dw_user, DWORD_PTR dw1, DWORD_PTR dw2);
				static void CALLBACK smc_timer_proc(UINT u_id, UINT u_msg, DWORD_PTR dw_user, DWORD_PTR dw1, DWORD_PTR dw2);
				static void write_data_to_file(std::vector<double>& graph_data);
    static void open_graph_window();

				CEdit m_time;
				CEdit m_Encoder1;
				CEdit m_Encoder2;
				CEdit m_Encoder3;
				CEdit m_statusTextBox;

				afx_msg void on_bn_clicked_initialize();
				afx_msg void on_bn_clicked_calibration();
				afx_msg void on_bn_clicked_read();
				afx_msg void on_bn_clicked_home();
				afx_msg void on_bn_clicked_smc();
};
