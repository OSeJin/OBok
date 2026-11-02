(* =====================================================================
   LS ELECTRIC XGI - 사용자 펑션블록 (ST)
   FB_Valve5PumpSeq : 밸브 5개 순차 개방 -> 펌프 기동 / 역순 정지

   XG5000 > 프로젝트 > 펑션/펑션블록 > 항목 추가 > 펑션블록(ST)
   아래 표를 FB 로컬 변수 테이블에 등록 후 본문을 붙여넣는다.
   인스턴스마다 내부 상태(스텝, 타이머)가 독립이므로 동일 설비 다수에 재사용 가능.

   [동작]
   1. Start 상승에지 -> V1 열림 지령 -> LS 확인 -> 간격 대기 -> V2 ... V5
   2. 5개 모두 개방 확인 -> 펌프 기동 -> 운전 피드백 확인 -> 운전
   3. Stop -> 펌프 정지 -> 정지 확인 -> V5 -> V1 역순 폐쇄
      (개방 도중 Stop 시 연 밸브까지만 역순 폐쇄)
   4. 고장 -> 펌프 즉시 정지 -> 지연 후 밸브 전체 폐쇄 -> Reset 상승에지로 복귀
   5. EStop_OK = FALSE -> 모든 출력 즉시 OFF (최우선)

   [변수 테이블]
   ---------------------------------------------------------------------
   종류        이름            타입                 초기값    설명
   ---------------------------------------------------------------------
   VAR_INPUT   Start           BOOL                           기동 (내부 상승에지)
   VAR_INPUT   Stop            BOOL                           정지 (레벨)
   VAR_INPUT   Reset           BOOL                           고장 리셋 (내부 상승에지)
   VAR_INPUT   EStop_OK        BOOL                           비상정지 정상 (B접점)
   VAR_INPUT   Pump_OL_OK      BOOL                           펌프 THR 정상 (B접점)
   VAR_INPUT   Pump_RunFb      BOOL                           펌프 MC 보조접점
   VAR_INPUT   V1_LS           BOOL                           밸브1 열림 LS
   VAR_INPUT   V2_LS           BOOL                           밸브2 열림 LS
   VAR_INPUT   V3_LS           BOOL                           밸브3 열림 LS
   VAR_INPUT   V4_LS           BOOL                           밸브4 열림 LS
   VAR_INPUT   V5_LS           BOOL                           밸브5 열림 LS
   VAR_INPUT   tVlvOpenTO      TIME                 T#30S     밸브 개방 확인 타임아웃
   VAR_INPUT   tVlvInterval    TIME                 T#2S      밸브 간 개/폐 간격
   VAR_INPUT   tPumpFbTO       TIME                 T#5S      펌프 피드백 타임아웃
   VAR_INPUT   tPumpStopWait   TIME                 T#3S      펌프 정지 후 밸브 폐쇄 대기
   VAR_INPUT   tFaultVlvDelay  TIME                 T#3S      고장 시 밸브 폐쇄 지연

   VAR_OUTPUT  V1_Open         BOOL                           밸브1 열림 지령
   VAR_OUTPUT  V2_Open         BOOL                           밸브2 열림 지령
   VAR_OUTPUT  V3_Open         BOOL                           밸브3 열림 지령
   VAR_OUTPUT  V4_Open         BOOL                           밸브4 열림 지령
   VAR_OUTPUT  V5_Open         BOOL                           밸브5 열림 지령
   VAR_OUTPUT  Pump_Run        BOOL                           펌프 MC 코일
   VAR_OUTPUT  Running         BOOL                           운전 중 (스텝 40)
   VAR_OUTPUT  Busy            BOOL                           기동/정지 시퀀스 진행 중
   VAR_OUTPUT  Fault           BOOL                           고장
   VAR_OUTPUT  FaultCode       INT                            고장 코드
   VAR_OUTPUT  Step            INT                            현재 스텝 (모니터링)

   VAR         iStep           INT                  0
   VAR         iStepPrev       INT                  0
   VAR         iIdx            INT                  0         처리 중 밸브 인덱스 0~4
   VAR         iChkMax         INT                  -1        개방 유지 감시 최대 인덱스
   VAR         i               INT                  0
   VAR         bVlvCmd         ARRAY[0..4] OF BOOL
   VAR         bVlvFb          ARRAY[0..4] OF BOOL
   VAR         bPumpCmd        BOOL
   VAR         StepTmr         TON                            스텝 경과시간
   VAR         R_Start         R_TRIG
   VAR         R_Reset         R_TRIG

   [스텝]
     0 : 대기            10 : 밸브 개방/LS 확인    20 : 밸브 간격 대기
    30 : 펌프 기동/확인    40 : 운전                50 : 펌프 정지 대기
    60 : 밸브 역순 폐쇄    61 : 폐쇄 인덱스 감소     90 : 고장

   [FaultCode]
     1~5   : 밸브 n 개방 타임아웃
     11~15 : 밸브 n 열림 LS 이탈 (개방 확인 후)
     20    : 펌프 운전 피드백 타임아웃
     21    : 운전 중 펌프 피드백 소실
     30    : 펌프 과부하 (THR)
     99    : 비상정지
   ===================================================================== *)

(* ---------- 1. 입력 처리 ---------- *)
bVlvFb[0] := V1_LS;
bVlvFb[1] := V2_LS;
bVlvFb[2] := V3_LS;
bVlvFb[3] := V4_LS;
bVlvFb[4] := V5_LS;

R_Start(CLK := Start);
R_Reset(CLK := Reset);

(* ---------- 2. 스텝 경과시간 타이머 ----------
   스텝 전환 다음 스캔에 IN=FALSE -> ET 리셋. CASE 밖에서 매 스캔 1회 호출. *)
StepTmr(IN := (iStep = iStepPrev), PT := T#1H);
iStepPrev := iStep;

(* ---------- 3. 공통 인터록 ---------- *)
IF NOT EStop_OK THEN
    FaultCode := 99;
    iStep := 90;
ELSIF NOT Pump_OL_OK AND iStep <> 90 THEN
    FaultCode := 30;
    iStep := 90;
END_IF;

(* ---------- 4. 개방 완료된 밸브 LS 이탈 감시 ---------- *)
CASE iStep OF
    10:         iChkMax := iIdx - 1;
    20, 30, 40: iChkMax := iIdx;
ELSE
    iChkMax := -1;
END_CASE;

FOR i := 0 TO iChkMax DO
    IF NOT bVlvFb[i] THEN
        FaultCode := 11 + i;
        iStep := 90;
    END_IF;
END_FOR;

(* ---------- 5. 메인 시퀀스 ---------- *)
CASE iStep OF

    0: (* 대기 *)
        bPumpCmd := FALSE;
        FOR i := 0 TO 4 DO
            bVlvCmd[i] := FALSE;
        END_FOR;
        iIdx := 0;
        IF R_Start.Q AND NOT Stop THEN
            iStep := 10;
        END_IF;

    10: (* 밸브 iIdx 개방 -> 열림 LS 확인 *)
        bVlvCmd[iIdx] := TRUE;
        IF Stop THEN
            iStep := 60;
        ELSIF bVlvFb[iIdx] THEN
            iStep := 20;
        ELSIF StepTmr.ET >= tVlvOpenTO THEN
            FaultCode := 1 + iIdx;
            iStep := 90;
        END_IF;

    20: (* 다음 밸브 전 간격 대기 *)
        IF Stop THEN
            iStep := 60;
        ELSIF StepTmr.ET >= tVlvInterval THEN
            IF iIdx >= 4 THEN
                iStep := 30;
            ELSE
                iIdx := iIdx + 1;
                iStep := 10;
            END_IF;
        END_IF;

    30: (* 펌프 기동 -> 피드백 확인 *)
        bPumpCmd := TRUE;
        IF Stop THEN
            iStep := 50;
        ELSIF Pump_RunFb THEN
            iStep := 40;
        ELSIF StepTmr.ET >= tPumpFbTO THEN
            FaultCode := 20;
            iStep := 90;
        END_IF;

    40: (* 운전 *)
        bPumpCmd := TRUE;
        IF Stop THEN
            iStep := 50;
        ELSIF NOT Pump_RunFb THEN
            FaultCode := 21;
            iStep := 90;
        END_IF;

    50: (* 펌프 정지 -> 정지 확인 + 대기 *)
        bPumpCmd := FALSE;
        IF NOT Pump_RunFb AND StepTmr.ET >= tPumpStopWait THEN
            iIdx := 4;
            iStep := 60;
        END_IF;

    60: (* 밸브 iIdx 폐쇄 -> 간격 대기 *)
        bPumpCmd := FALSE;
        bVlvCmd[iIdx] := FALSE;
        IF StepTmr.ET >= tVlvInterval THEN
            iStep := 61;
        END_IF;

    61: (* 폐쇄 인덱스 감소 (1스캔) *)
        IF iIdx <= 0 THEN
            iStep := 0;
        ELSE
            iIdx := iIdx - 1;
            iStep := 60;
        END_IF;

    90: (* 고장 *)
        bPumpCmd := FALSE;
        IF StepTmr.ET >= tFaultVlvDelay THEN
            FOR i := 0 TO 4 DO
                bVlvCmd[i] := FALSE;
            END_FOR;
        END_IF;
        IF R_Reset.Q AND EStop_OK AND Pump_OL_OK AND NOT Pump_RunFb THEN
            FaultCode := 0;
            iStep := 0;
        END_IF;

ELSE
    iStep := 90;
END_CASE;

(* ---------- 6. 비상정지 하드 오버라이드 ---------- *)
IF NOT EStop_OK THEN
    bPumpCmd := FALSE;
    FOR i := 0 TO 4 DO
        bVlvCmd[i] := FALSE;
    END_FOR;
END_IF;

(* ---------- 7. 출력 ---------- *)
V1_Open  := bVlvCmd[0];
V2_Open  := bVlvCmd[1];
V3_Open  := bVlvCmd[2];
V4_Open  := bVlvCmd[3];
V5_Open  := bVlvCmd[4];
Pump_Run := bPumpCmd;
Running  := (iStep = 40);
Busy     := (iStep <> 0) AND (iStep <> 40) AND (iStep <> 90);
Fault    := (iStep = 90);
Step     := iStep;
