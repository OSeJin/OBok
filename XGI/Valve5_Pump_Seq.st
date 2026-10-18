(* =====================================================================
   LS ELECTRIC XGI - ST 프로그램
   밸브 5개 순차 개방 -> 펌프 기동 / 정지 시 역순 정지 시퀀스

   [동작 개요]
   1. 기동 PB 상승에지 -> V1 열림 지령 -> V1 열림 LS 확인 -> 간격 대기
      -> V2 ... V5 동일 반복 (개방 타임아웃 시 고장)
   2. 5개 모두 개방 확인 -> 펌프 기동 -> 운전 피드백(MC 보조접점) 확인
   3. 운전 중 밸브 LS 이탈 / 펌프 피드백 소실 / OL 트립 -> 고장
   4. 정지 PB -> 펌프 정지 -> 정지 확인 -> V5 -> V1 역순 폐쇄
      (개방 중 정지 시 현재까지 연 밸브만 역순 폐쇄)
   5. 고장 -> 펌프 즉시 정지 -> 지연 후 밸브 전체 폐쇄 -> 리셋 PB로 복귀
   6. 비상정지(B접점) -> 모든 출력 즉시 OFF (스텝 무관 최우선)

   [변수 테이블] - XGI 글로벌/로컬 변수 테이블에 아래와 같이 등록
   ---------------------------------------------------------------------
   이름              타입              주소/초기값     설명
   ---------------------------------------------------------------------
   I_Start_PB        BOOL              %IX0.0.0        기동 PB (A접점)
   I_Stop_PB         BOOL              %IX0.0.1        정지 PB (A접점)
   I_Reset_PB        BOOL              %IX0.0.2        고장 리셋 PB (A접점)
   I_EStop           BOOL              %IX0.0.3        비상정지 (B접점, TRUE=정상)
   I_Pump_OL         BOOL              %IX0.0.4        펌프 THR (B접점, TRUE=정상)
   I_Pump_Run        BOOL              %IX0.0.5        펌프 MC 보조접점 (운전 피드백)
   I_V1_Open_LS      BOOL              %IX0.0.8        밸브1 열림 LS
   I_V2_Open_LS      BOOL              %IX0.0.9        밸브2 열림 LS
   I_V3_Open_LS      BOOL              %IX0.0.10       밸브3 열림 LS
   I_V4_Open_LS      BOOL              %IX0.0.11       밸브4 열림 LS
   I_V5_Open_LS      BOOL              %IX0.0.12       밸브5 열림 LS

   Q_V1_Open         BOOL              %QX0.1.0        밸브1 열림 지령
   Q_V2_Open         BOOL              %QX0.1.1        밸브2 열림 지령
   Q_V3_Open         BOOL              %QX0.1.2        밸브3 열림 지령
   Q_V4_Open         BOOL              %QX0.1.3        밸브4 열림 지령
   Q_V5_Open         BOOL              %QX0.1.4        밸브5 열림 지령
   Q_Pump_Run        BOOL              %QX0.1.5        펌프 MC 코일
   Q_Run_Lamp        BOOL              %QX0.1.6        운전 표시등
   Q_Fault_Lamp      BOOL              %QX0.1.7        고장 표시등 (점멸)

   (* 로컬 변수 *)
   iStep             INT               0               시퀀스 스텝
   iStepPrev         INT               0               이전 스캔 스텝 (스텝 타이머 리셋용)
   iIdx              INT               0               현재 처리 중인 밸브 인덱스 (0~4)
   iChkMax           INT               -1              개방 유지 감시 대상 최대 인덱스
   i                 INT               0               FOR 루프 카운터
   iFaultCode        INT               0               고장 코드 (하단 표 참조)
   bVlvCmd           ARRAY[0..4] OF BOOL               밸브 지령 내부 배열
   bVlvFb            ARRAY[0..4] OF BOOL               밸브 열림 LS 내부 배열
   bPumpCmd          BOOL              FALSE           펌프 지령 내부
   StepTmr           TON                               스텝 경과시간 타이머
   FaultBlink        TON                               고장등 점멸 타이머
   bBlink            BOOL              FALSE           점멸 비트
   R_Start           R_TRIG                            기동 PB 상승에지
   R_Reset           R_TRIG                            리셋 PB 상승에지

   tVlvOpenTO        TIME              T#30S           밸브 개방 확인 타임아웃
   tVlvInterval      TIME              T#2S            밸브 간 개/폐 간격
   tPumpFbTO         TIME              T#5S            펌프 운전 피드백 타임아웃
   tPumpStopWait     TIME              T#3S            펌프 정지 후 밸브 폐쇄 전 대기
   tFaultVlvDelay    TIME              T#3S            고장 시 펌프 정지 후 밸브 폐쇄 지연

   [스텝]
     0 : 대기          10 : 밸브 개방/LS 확인    20 : 밸브 간격 대기
    30 : 펌프 기동/확인  40 : 운전                50 : 펌프 정지 대기
    60 : 밸브 역순 폐쇄  61 : 폐쇄 인덱스 감소     90 : 고장

   [고장 코드]
     1~5   : 밸브 n 개방 타임아웃
     11~15 : 밸브 n 열림 LS 이탈 (개방 후)
     20    : 펌프 운전 피드백 타임아웃
     21    : 운전 중 펌프 피드백 소실
     30    : 펌프 과부하 (THR)
     99    : 비상정지
   ===================================================================== *)

(* ---------- 1. 입력 매핑 ---------- *)
bVlvFb[0] := I_V1_Open_LS;
bVlvFb[1] := I_V2_Open_LS;
bVlvFb[2] := I_V3_Open_LS;
bVlvFb[3] := I_V4_Open_LS;
bVlvFb[4] := I_V5_Open_LS;

R_Start(CLK := I_Start_PB);
R_Reset(CLK := I_Reset_PB);

(* ---------- 2. 스텝 경과시간 타이머 ----------
   스텝이 바뀐 다음 스캔에 IN=FALSE로 ET를 0으로 리셋.
   타이머 인스턴스는 CASE 밖에서 매 스캔 1회만 호출한다. *)
StepTmr(IN := (iStep = iStepPrev), PT := T#1H);
iStepPrev := iStep;

(* ---------- 3. 공통 인터록 (최우선) ---------- *)
IF NOT I_EStop THEN
    iFaultCode := 99;
    iStep := 90;
ELSIF NOT I_Pump_OL AND iStep <> 90 THEN
    iFaultCode := 30;
    iStep := 90;
END_IF;

(* ---------- 4. 개방 완료된 밸브의 LS 이탈 감시 ---------- *)
CASE iStep OF
    10:         iChkMax := iIdx - 1;
    20, 30, 40: iChkMax := iIdx;
ELSE
    iChkMax := -1;
END_CASE;

FOR i := 0 TO iChkMax DO
    IF NOT bVlvFb[i] THEN
        iFaultCode := 11 + i;
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
        IF R_Start.Q AND NOT I_Stop_PB THEN
            iStep := 10;
        END_IF;

    10: (* 밸브 iIdx 개방 지령 -> 열림 LS 확인 *)
        bVlvCmd[iIdx] := TRUE;
        IF I_Stop_PB THEN
            iStep := 60;
        ELSIF bVlvFb[iIdx] THEN
            iStep := 20;
        ELSIF StepTmr.ET >= tVlvOpenTO THEN
            iFaultCode := 1 + iIdx;
            iStep := 90;
        END_IF;

    20: (* 다음 밸브 전 간격 대기 *)
        IF I_Stop_PB THEN
            iStep := 60;
        ELSIF StepTmr.ET >= tVlvInterval THEN
            IF iIdx >= 4 THEN
                iStep := 30;            (* iIdx = 4 유지 -> 30/40에서 전체 감시 *)
            ELSE
                iIdx := iIdx + 1;
                iStep := 10;
            END_IF;
        END_IF;

    30: (* 펌프 기동 -> 운전 피드백 확인 *)
        bPumpCmd := TRUE;
        IF I_Stop_PB THEN
            iStep := 50;
        ELSIF I_Pump_Run THEN
            iStep := 40;
        ELSIF StepTmr.ET >= tPumpFbTO THEN
            iFaultCode := 20;
            iStep := 90;
        END_IF;

    40: (* 운전 *)
        bPumpCmd := TRUE;
        IF I_Stop_PB THEN
            iStep := 50;
        ELSIF NOT I_Pump_Run THEN
            iFaultCode := 21;
            iStep := 90;
        END_IF;

    50: (* 펌프 정지 -> 정지 확인 + 대기 후 밸브 폐쇄 *)
        bPumpCmd := FALSE;
        IF NOT I_Pump_Run AND StepTmr.ET >= tPumpStopWait THEN
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

    90: (* 고장: 펌프 즉시 정지 -> 지연 후 밸브 전체 폐쇄 *)
        bPumpCmd := FALSE;
        IF StepTmr.ET >= tFaultVlvDelay THEN
            FOR i := 0 TO 4 DO
                bVlvCmd[i] := FALSE;
            END_FOR;
        END_IF;
        IF R_Reset.Q AND I_EStop AND I_Pump_OL AND NOT I_Pump_Run THEN
            iFaultCode := 0;
            iStep := 0;
        END_IF;

ELSE
    iStep := 90;    (* 정의되지 않은 스텝 방어 *)
END_CASE;

(* ---------- 6. 비상정지 하드 오버라이드 ---------- *)
IF NOT I_EStop THEN
    bPumpCmd := FALSE;
    FOR i := 0 TO 4 DO
        bVlvCmd[i] := FALSE;
    END_FOR;
END_IF;

(* ---------- 7. 표시등 ---------- *)
FaultBlink(IN := NOT FaultBlink.Q, PT := T#500MS);
IF FaultBlink.Q THEN
    bBlink := NOT bBlink;
END_IF;

(* ---------- 8. 출력 매핑 ---------- *)
Q_V1_Open    := bVlvCmd[0];
Q_V2_Open    := bVlvCmd[1];
Q_V3_Open    := bVlvCmd[2];
Q_V4_Open    := bVlvCmd[3];
Q_V5_Open    := bVlvCmd[4];
Q_Pump_Run   := bPumpCmd;
Q_Run_Lamp   := (iStep = 40);
Q_Fault_Lamp := (iStep = 90) AND bBlink;
