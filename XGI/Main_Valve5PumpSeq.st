(* =====================================================================
   LS ELECTRIC XGI - 스캔 프로그램 (ST)
   FB_Valve5PumpSeq 인스턴스 호출 예시

   [글로벌 변수 테이블]
   ---------------------------------------------------------------------
   이름              타입               주소          설명
   ---------------------------------------------------------------------
   I_Start_PB        BOOL               %IX0.0.0      기동 PB (A접점)
   I_Stop_PB         BOOL               %IX0.0.1      정지 PB (A접점)
   I_Reset_PB        BOOL               %IX0.0.2      리셋 PB (A접점)
   I_EStop           BOOL               %IX0.0.3      비상정지 (B접점)
   I_Pump_OL         BOOL               %IX0.0.4      펌프 THR (B접점)
   I_Pump_Run        BOOL               %IX0.0.5      펌프 MC 보조접점
   I_V1_Open_LS      BOOL               %IX0.0.8      밸브1 열림 LS
   I_V2_Open_LS      BOOL               %IX0.0.9      밸브2 열림 LS
   I_V3_Open_LS      BOOL               %IX0.0.10     밸브3 열림 LS
   I_V4_Open_LS      BOOL               %IX0.0.11     밸브4 열림 LS
   I_V5_Open_LS      BOOL               %IX0.0.12     밸브5 열림 LS

   Q_V1_Open         BOOL               %QX0.1.0      밸브1 열림 지령
   Q_V2_Open         BOOL               %QX0.1.1      밸브2 열림 지령
   Q_V3_Open         BOOL               %QX0.1.2      밸브3 열림 지령
   Q_V4_Open         BOOL               %QX0.1.3      밸브4 열림 지령
   Q_V5_Open         BOOL               %QX0.1.4      밸브5 열림 지령
   Q_Pump_Run        BOOL               %QX0.1.5      펌프 MC 코일
   Q_Run_Lamp        BOOL               %QX0.1.6      운전 표시등
   Q_Fault_Lamp      BOOL               %QX0.1.7      고장 표시등 (1초 점멸)

   [프로그램 로컬 변수]
   Seq1              FB_Valve5PumpSeq                 시퀀스 인스턴스
   ===================================================================== *)

Seq1(
    Start          := I_Start_PB,
    Stop           := I_Stop_PB,
    Reset          := I_Reset_PB,
    EStop_OK       := I_EStop,
    Pump_OL_OK     := I_Pump_OL,
    Pump_RunFb     := I_Pump_Run,
    V1_LS          := I_V1_Open_LS,
    V2_LS          := I_V2_Open_LS,
    V3_LS          := I_V3_Open_LS,
    V4_LS          := I_V4_Open_LS,
    V5_LS          := I_V5_Open_LS,
    tVlvOpenTO     := T#30S,
    tVlvInterval   := T#2S,
    tPumpFbTO      := T#5S,
    tPumpStopWait  := T#3S,
    tFaultVlvDelay := T#3S);

Q_V1_Open    := Seq1.V1_Open;
Q_V2_Open    := Seq1.V2_Open;
Q_V3_Open    := Seq1.V3_Open;
Q_V4_Open    := Seq1.V4_Open;
Q_V5_Open    := Seq1.V5_Open;
Q_Pump_Run   := Seq1.Pump_Run;
Q_Run_Lamp   := Seq1.Running OR (Seq1.Busy AND _T1S);
Q_Fault_Lamp := Seq1.Fault AND _T1S;
