(* =====================================================================
   LS ELECTRIC XGI - ST 프로그램
   RS-485 Modbus RTU 마스터 (폴링 테이블 / CRC16 / 타임아웃 / 재시도)

   [동작 개요]
   1. 잡 테이블(최대 8개)을 순차 폴링한다.
      FC03 : Holding Register 읽기 (최대 120워드)
      FC06 : Single Register 쓰기
      FC16 : Multiple Register 쓰기 (최대 120워드)
   2. 잡 모드 0 = 상시 순환, 1 = 요청 시 1회 (aJobReq 상승 후 완료/최종실패 시 자동 클리어)
   3. 송신 프레임 조립 + CRC16(Modbus, 다항식 16#A001, Low 바이트 먼저) 부가
   4. 응답은 기대 길이(FC03 = 5+2N, FC06/16 = 8, 예외 = 5)에 도달하면 즉시 해석
      -> 3.5문자 무음시간 판정에 의존하지 않음 (스캔타임 영향 없음)
   5. 검증 : CRC / 국번 / FC / 바이트수 또는 에코 일치 / 예외응답(100+예외코드)
   6. 실패 시 iRetryMax 회 재시도 -> 최종 실패면 해당 잡 고장 비트 ON
      (다른 잡은 계속 폴링, 한 국 단선이 전체 통신을 막지 않음)
   7. 프레임 간 tGap 이상 버스 휴지 (RS-485 반이중 턴어라운드 + 3.5문자 확보)

   [포트 인터페이스] - P_ 변수는 실제 RS-485 포트 드라이버와 연결한다.
   Cnet(XGL-C42A 등) P2P 사용자 정의 프레임 / 내장 Cnet 등 사용하는 포트의
   송수신 블록에 아래 핸드셰이크로 결선하면 프로토콜 계층은 그대로 재사용된다.
     P_TxReq  (PLC->드라이버) : TRUE 동안 P_TxBuf[0..P_TxLen-1] 송신 요청
     P_TxDone (드라이버->PLC) : 송신 완료. P_TxReq FALSE 시 드라이버가 FALSE로 복귀
     P_RxClr  (PLC->드라이버) : TRUE 동안 수신 버퍼/카운트 클리어
     P_RxCnt  (드라이버->PLC) : P_RxClr 해제 이후 수신 누적 바이트 수
     P_RxBuf  (드라이버->PLC) : 수신 바이트 (0번부터 누적)
   ※ 포트 설정 : 9600~115200bps, 8bit, None/Even, 1stop - 슬레이브와 동일하게
   ※ 배선 : 버스 양 끝단 120Ω 종단, 바이어스 1개소, 쉴드 1점 접지, SG 공통 연결

   [변수 테이블] - XGI 글로벌/로컬 변수 테이블에 아래와 같이 등록
   ---------------------------------------------------------------------
   이름              타입                     초기값         설명
   ---------------------------------------------------------------------
   (* 포트 인터페이스 (글로벌) *)
   P_TxBuf           ARRAY[0..259] OF BYTE                   송신 버퍼
   P_TxLen           INT                      0              송신 바이트 수
   P_TxReq           BOOL                     FALSE          송신 요청
   P_TxDone          BOOL                     FALSE          송신 완료
   P_RxBuf           ARRAY[0..259] OF BYTE                   수신 버퍼
   P_RxCnt           INT                      0              수신 바이트 수
   P_RxClr           BOOL                     FALSE          수신 버퍼 클리어

   (* 잡 테이블 (글로벌, HMI 연동 가능) *)
   aJobSlave         ARRAY[0..7] OF BYTE                     국번 1~247 (0 = 미사용)
   aJobFc            ARRAY[0..7] OF BYTE                     3 / 6 / 16
   aJobAddr          ARRAY[0..7] OF WORD                     시작 주소 (0 기준, 40001 -> 0)
   aJobQty           ARRAY[0..7] OF INT                      워드 수 (FC06은 무시, 1 고정)
   aJobOfs           ARRAY[0..7] OF INT                      awData 내 시작 인덱스
   aJobMode          ARRAY[0..7] OF INT                      0 = 상시, 1 = 요청 시 1회
   aJobReq           ARRAY[0..7] OF BOOL                     1회 실행 요청 (모드 1)
   aJobErr           ARRAY[0..7] OF INT                      최근 결과 코드 (하단 표)
   aJobFault         ARRAY[0..7] OF BOOL                     재시도 후 최종 실패
   awData            ARRAY[0..255] OF WORD                   읽기 결과 / 쓰기 원본 데이터

   (* 상태 (글로벌) *)
   bCommEn           BOOL                     TRUE           통신 허가
   bCommFault        BOOL                     FALSE          사용 중 잡 중 하나라도 고장
   dTxCnt            DINT                     0              송신 트랜잭션 수
   dOkCnt            DINT                     0              정상 응답 수
   dErrCnt           DINT                     0              오류 응답 수 (재시도 포함)

   (* 로컬 변수 *)
   bInit             BOOL                     FALSE          잡 테이블 초기화 완료
   iStep             INT                      0              시퀀스 스텝
   iStepPrev         INT                      0              이전 스캔 스텝 (스텝 타이머 리셋용)
   iJob              INT                      7              현재 잡 인덱스 (7 -> 첫 탐색 시 0)
   iRetry            INT                      0              현재 잡 재시도 횟수
   iErr              INT                      0              현재 트랜잭션 결과 코드
   iFc               INT                      0              현재 FC
   iQty              INT                      0              현재 워드 수
   iOfs              INT                      0              현재 데이터 오프셋
   iExpLen           INT                      0              기대 응답 길이
   i                 INT                      0              루프 카운터
   j                 INT                      0              루프 카운터
   k                 INT                      0              루프 카운터
   bFound            BOOL                     FALSE          다음 잡 탐색 결과
   wCrc              WORD                     0              CRC 계산용
   wTmp              WORD                     0              작업용
   StepTmr           TON                                     스텝 경과시간 타이머

   iRetryMax         INT                      2              재시도 횟수
   tGap              TIME                     T#20MS         프레임 간 버스 휴지시간
   tTxTO             TIME                     T#500MS        드라이버 송신 완료 타임아웃
   tRespTO           TIME                     T#300MS        슬레이브 응답 타임아웃

   [스텝]
     0 : 다음 잡 탐색      10 : 프레임 조립/CRC     20 : 버스 휴지/수신 클리어 대기
    30 : 송신 완료 대기    40 : 응답 수신 대기      50 : 응답 검증/데이터 저장
    60 : 재시도 판정       80 : 잡 결과 확정

   [결과 코드 aJobErr]
     0       : 정상
     1       : 응답 타임아웃 (무응답 / 국번 오류 / 배선)
     2       : 드라이버 송신 또는 수신 클리어 타임아웃
     3       : CRC 오류 (노이즈 / 종단 / 통신속도 불일치)
     4       : 응답 국번 불일치
     5       : 응답 FC 불일치
     6       : 바이트수 또는 에코 불일치
     7       : 잡 설정 오류 (FC / 수량 / 오프셋 범위)
     101~    : 슬레이브 예외응답 (100 + 예외코드, 102 = 주소 오류 등)
   ===================================================================== *)

(* ---------- 1. 잡 테이블 초기화 (첫 스캔 1회, 현장에 맞게 수정) ---------- *)
IF NOT bInit THEN
    (* 잡0 : 1국 40001~40010 읽기 -> awData[0..9] *)
    aJobSlave[0] := 1;  aJobFc[0] := 3;  aJobAddr[0] := 0;    aJobQty[0] := 10; aJobOfs[0] := 0;  aJobMode[0] := 0;
    (* 잡1 : 2국 40101~40104 읽기 -> awData[10..13] *)
    aJobSlave[1] := 2;  aJobFc[1] := 3;  aJobAddr[1] := 100;  aJobQty[1] := 4;  aJobOfs[1] := 10; aJobMode[1] := 0;
    (* 잡2 : 1국 40021 쓰기 <- awData[20], 요청 시 1회 *)
    aJobSlave[2] := 1;  aJobFc[2] := 6;  aJobAddr[2] := 20;   aJobQty[2] := 1;  aJobOfs[2] := 20; aJobMode[2] := 1;
    (* 잡3 : 2국 40201~40204 쓰기 <- awData[30..33], 상시 *)
    aJobSlave[3] := 2;  aJobFc[3] := 16; aJobAddr[3] := 200;  aJobQty[3] := 4;  aJobOfs[3] := 30; aJobMode[3] := 0;
    bInit := TRUE;
END_IF;

(* ---------- 2. 스텝 경과시간 타이머 ----------
   스텝이 바뀐 다음 스캔에 IN=FALSE로 ET를 0으로 리셋.
   타이머 인스턴스는 CASE 밖에서 매 스캔 1회만 호출한다. *)
StepTmr(IN := (iStep = iStepPrev), PT := T#1H);
iStepPrev := iStep;

(* ---------- 3. 통신 금지 시 대기 복귀 ---------- *)
IF NOT bCommEn THEN
    iStep := 0;
END_IF;

(* ---------- 4. 메인 시퀀스 ---------- *)
CASE iStep OF

    0: (* 다음 실행 가능 잡 탐색 (현재 잡 다음부터 순환) *)
        IF bCommEn THEN
            bFound := FALSE;
            FOR i := 1 TO 8 DO
                IF NOT bFound THEN
                    k := (iJob + i) MOD 8;
                    IF aJobSlave[k] <> 0 AND (aJobMode[k] = 0 OR aJobReq[k]) THEN
                        iJob := k;
                        bFound := TRUE;
                    END_IF;
                END_IF;
            END_FOR;
            IF bFound THEN
                iRetry := 0;
                iStep := 10;
            END_IF;
        END_IF;

    10: (* 잡 설정 검증 -> 요청 프레임 조립 -> CRC 부가 *)
        iFc  := BYTE_TO_INT(aJobFc[iJob]);
        iQty := aJobQty[iJob];
        iOfs := aJobOfs[iJob];
        IF iFc = 6 THEN
            iQty := 1;
        END_IF;

        IF (iFc <> 3 AND iFc <> 6 AND iFc <> 16)
           OR iQty < 1 OR iQty > 120
           OR iOfs < 0 OR iOfs + iQty > 256
           OR BYTE_TO_INT(aJobSlave[iJob]) > 247 THEN
            iErr := 7;
            iStep := 80;
        ELSE
            P_TxBuf[0] := aJobSlave[iJob];
            P_TxBuf[1] := aJobFc[iJob];
            P_TxBuf[2] := WORD_TO_BYTE(SHR(aJobAddr[iJob], 8));
            P_TxBuf[3] := WORD_TO_BYTE(aJobAddr[iJob]);

            CASE iFc OF
                3:
                    P_TxBuf[4] := INT_TO_BYTE(iQty / 256);
                    P_TxBuf[5] := INT_TO_BYTE(iQty MOD 256);
                    P_TxLen := 6;
                    iExpLen := 5 + iQty * 2;
                6:
                    P_TxBuf[4] := WORD_TO_BYTE(SHR(awData[iOfs], 8));
                    P_TxBuf[5] := WORD_TO_BYTE(awData[iOfs]);
                    P_TxLen := 6;
                    iExpLen := 8;
                16:
                    P_TxBuf[4] := INT_TO_BYTE(iQty / 256);
                    P_TxBuf[5] := INT_TO_BYTE(iQty MOD 256);
                    P_TxBuf[6] := INT_TO_BYTE(iQty * 2);
                    FOR i := 0 TO iQty - 1 DO
                        P_TxBuf[7 + i * 2] := WORD_TO_BYTE(SHR(awData[iOfs + i], 8));
                        P_TxBuf[8 + i * 2] := WORD_TO_BYTE(awData[iOfs + i]);
                    END_FOR;
                    P_TxLen := 7 + iQty * 2;
                    iExpLen := 8;
            END_CASE;

            (* CRC16 Modbus *)
            wCrc := 16#FFFF;
            FOR i := 0 TO P_TxLen - 1 DO
                wCrc := wCrc XOR BYTE_TO_WORD(P_TxBuf[i]);
                FOR j := 0 TO 7 DO
                    IF (wCrc AND 16#0001) <> 0 THEN
                        wCrc := SHR(wCrc, 1) XOR 16#A001;
                    ELSE
                        wCrc := SHR(wCrc, 1);
                    END_IF;
                END_FOR;
            END_FOR;
            P_TxBuf[P_TxLen]     := WORD_TO_BYTE(wCrc);              (* CRC Low *)
            P_TxBuf[P_TxLen + 1] := WORD_TO_BYTE(SHR(wCrc, 8));      (* CRC High *)
            P_TxLen := P_TxLen + 2;

            iErr := 0;
            iStep := 20;
        END_IF;

    20: (* 버스 휴지시간 확보 + 수신 버퍼 클리어 확인 (P_RxClr = TRUE) *)
        IF StepTmr.ET >= tGap AND P_RxCnt = 0 THEN
            iStep := 30;
        ELSIF StepTmr.ET >= tTxTO THEN
            iErr := 2;
            iStep := 60;
        END_IF;

    30: (* 송신 요청 (P_TxReq = TRUE) -> 송신 완료 대기 *)
        IF P_TxDone THEN
            dTxCnt := dTxCnt + 1;
            iStep := 40;
        ELSIF StepTmr.ET >= tTxTO THEN
            iErr := 2;
            iStep := 60;
        END_IF;

    40: (* 응답 수신 대기 - 예외응답이면 기대 길이 5로 전환 *)
        IF P_RxCnt >= 2 THEN
            IF (BYTE_TO_WORD(P_RxBuf[1]) AND 16#0080) <> 0 THEN
                iExpLen := 5;
            END_IF;
        END_IF;
        IF P_RxCnt >= iExpLen THEN
            iStep := 50;
        ELSIF StepTmr.ET >= tRespTO THEN
            iErr := 1;
            iStep := 60;
        END_IF;

    50: (* 응답 검증 -> 데이터 저장 *)
        wCrc := 16#FFFF;
        FOR i := 0 TO iExpLen - 3 DO
            wCrc := wCrc XOR BYTE_TO_WORD(P_RxBuf[i]);
            FOR j := 0 TO 7 DO
                IF (wCrc AND 16#0001) <> 0 THEN
                    wCrc := SHR(wCrc, 1) XOR 16#A001;
                ELSE
                    wCrc := SHR(wCrc, 1);
                END_IF;
            END_FOR;
        END_FOR;

        IF P_RxBuf[iExpLen - 2] <> WORD_TO_BYTE(wCrc)
           OR P_RxBuf[iExpLen - 1] <> WORD_TO_BYTE(SHR(wCrc, 8)) THEN
            iErr := 3;
        ELSIF P_RxBuf[0] <> P_TxBuf[0] THEN
            iErr := 4;
        ELSIF P_RxBuf[1] = (P_TxBuf[1] OR 16#80) THEN
            iErr := 100 + BYTE_TO_INT(P_RxBuf[2]);
        ELSIF P_RxBuf[1] <> P_TxBuf[1] THEN
            iErr := 5;
        ELSE
            CASE iFc OF
                3:
                    IF BYTE_TO_INT(P_RxBuf[2]) <> iQty * 2 THEN
                        iErr := 6;
                    ELSE
                        FOR i := 0 TO iQty - 1 DO
                            wTmp := SHL(BYTE_TO_WORD(P_RxBuf[3 + i * 2]), 8);
                            awData[iOfs + i] := wTmp OR BYTE_TO_WORD(P_RxBuf[4 + i * 2]);
                        END_FOR;
                    END_IF;
                6:  (* 요청 프레임 전체 에코 *)
                    FOR i := 2 TO 5 DO
                        IF P_RxBuf[i] <> P_TxBuf[i] THEN
                            iErr := 6;
                        END_IF;
                    END_FOR;
                16: (* 시작주소 + 수량 에코 *)
                    FOR i := 2 TO 5 DO
                        IF P_RxBuf[i] <> P_TxBuf[i] THEN
                            iErr := 6;
                        END_IF;
                    END_FOR;
            END_CASE;
        END_IF;

        IF iErr = 0 THEN
            dOkCnt := dOkCnt + 1;
            iStep := 80;
        ELSE
            iStep := 60;
        END_IF;

    60: (* 재시도 판정 *)
        dErrCnt := dErrCnt + 1;
        IF iRetry < iRetryMax THEN
            iRetry := iRetry + 1;
            iStep := 10;               (* 쓰기 잡은 최신 awData로 재조립 *)
        ELSE
            iStep := 80;
        END_IF;

    80: (* 잡 결과 확정 *)
        aJobErr[iJob]   := iErr;
        aJobFault[iJob] := (iErr <> 0);
        IF aJobMode[iJob] = 1 THEN
            aJobReq[iJob] := FALSE;
        END_IF;
        iStep := 0;

ELSE
    iStep := 0;     (* 정의되지 않은 스텝 방어 *)
END_CASE;

(* ---------- 5. 통신 고장 종합 (미사용 잡 제외) ---------- *)
bCommFault := FALSE;
FOR i := 0 TO 7 DO
    IF aJobSlave[i] <> 0 AND aJobFault[i] THEN
        bCommFault := TRUE;
    END_IF;
END_FOR;

(* ---------- 6. 포트 핸드셰이크 출력 ---------- *)
P_RxClr := (iStep = 10) OR (iStep = 20);
P_TxReq := (iStep = 30);
