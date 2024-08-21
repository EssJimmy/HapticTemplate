
// HapticTemplateDlg.h : header file
//

#pragma once
#include "pch.h"
#include "framework.h"
#include "stdafx.h"
#include "HapticTemplate.h"
#include <afxwin.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <GL/glut.h>

// CHapticTemplateDlg dialog
class CHapticTemplateDlg : public CDialogEx
{
// Construction
public:
	CHapticTemplateDlg(CWnd* pParent = nullptr);	// standard constructor

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_HAPTICTEMPLATE_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg void OnClose();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()

public:
    static void CALLBACK HomeTimerProc(UINT uID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2);
				static void CALLBACK SmcTimerProc(UINT uID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2);

				CEdit m_time;
				CEdit m_Encoder1;
				CEdit m_Encoder2;
				CEdit m_Encoder3;
				CEdit m_statusTextBox;

				afx_msg void OnBnClickedInitialize();
				afx_msg void OnBnClickedCalib();
				afx_msg void OnBnClickedReed();
				afx_msg void OnBnClickedHome();
				afx_msg void OnBnClickedSmc();
};
