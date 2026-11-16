/* Compile the ACTUAL firmware control source against passive HAL stubs. */
#include <assert.h>
#include <stdio.h>
#include "../App/Src/claw_control.c"

static void reset(void) { test_tick = 100; ClawControl_Init(); test_tick += CLAW_DIRECTION_QUIET_MS; }
static void tick(unsigned ms, int heartbeat)
{
  test_tick += ms;
  if (heartbeat) last_command_tick = test_tick;
  ClawControl_Task();
}
static void edge(unsigned m, int counts)
{
  motors[m].position_counts += counts;
  motors[m].last_hall_tick = test_tick;
}

static void test_jog(void)
{
  ClawCommand jog = {20, CLAW_COMMAND_JOG, 0, 0, 1};
  ClawCommand stop = {21, CLAW_COMMAND_STOP, 0, 0, 0};
  reset();
  assert(ClawControl_HandleCommand(&jog) == CLAW_RESULT_OK);
  tick(10, 0); assert(motors[0].duty_permille == 0);
  tick(10, 0); assert(motors[0].duty_permille == 200);
  assert(motors[1].duty_permille == 0 && motors[2].duty_permille == 0);
  tick(100, 0);
  assert(ClawControl_HandleCommand(&jog) == CLAW_RESULT_OK); /* duplicate does not restart */
  tick(100, 0); assert(claw_state == CLAW_STATE_IDLE && !jog_active);
  assert(motors[0].duty_permille == 0);
  jog.sequence++;
  assert(ClawControl_HandleCommand(&jog) == CLAW_RESULT_BUSY);
  tick(300, 0); jog.sequence++;
  assert(ClawControl_HandleCommand(&jog) == CLAW_RESULT_OK);
  tick(20, 0); edge(0, -10); tick(10, 0);
  assert(claw_state == CLAW_STATE_IDLE && motors[0].duty_permille == 0);
  reset(); assert(ClawControl_HandleCommand(&jog) == CLAW_RESULT_OK);
  tick(20, 0); assert(ClawControl_HandleCommand(&stop) == CLAW_RESULT_OK);
  tick(50, 0); assert(!jog_active && motors[0].duty_permille == 0);
  reset(); assert(ClawControl_HandleCommand(&jog) == CLAW_RESULT_OK);
  tick(20, 0); ClawControl_TransportFault(); tick(50, 0);
  assert(claw_state == CLAW_STATE_FAULT && !jog_active && motors[0].duty_permille == 0);
  reset(); jog.flags = 2;
  assert(ClawControl_HandleCommand(&jog) == CLAW_RESULT_RANGE);
  reset(); jog.flags = 0; jog.speed_drpm = 0;
  assert(ClawControl_HandleCommand(&jog) == CLAW_RESULT_RANGE);
  puts("Fixed jog: duration, isolation, duplicate, cooldown, displacement, stop/fault checks passed");
  reset(); jog.flags = 0; jog.speed_drpm = 3;
  assert(ClawControl_HandleCommand(&jog) == CLAW_RESULT_OK);
  tick(20, 0); edge(2, 10); tick(10, 0);
  if (CLAW_PROFILE_GROUP == 2)
  {
    assert(jog_active && motors[2].duty_permille == 200);
    assert(motors[0].duty_permille == 0 && motors[1].duty_permille == 0);
    edge(2, 19); tick(10, 0); assert(jog_active);
    edge(2, 1); tick(10, 0);
  }
  assert(!jog_active && motors[2].duty_permille == 0);
  reset(); assert(ClawControl_HandleCommand(&jog) == CLAW_RESULT_OK);
  tick(20, 0); tick(200, 0);
  assert(!jog_active && motors[2].duty_permille == 0);
  reset(); jog.flags = 1;
  assert(ClawControl_HandleCommand(&jog) == CLAW_RESULT_OK);
  tick(20, 0); edge(2, -10); tick(10, 0);
  assert(!jog_active && motors[2].duty_permille == 0);
}

static void test_seek(void)
{
  ClawCommand seek = {70, CLAW_COMMAND_SEEK_OPEN, 0, 0, 1};
  unsigned m = 0;
  if (CLAW_COMMISSIONED_MASK == 2) m=1;
  if (CLAW_COMMISSIONED_MASK == 4) m=2;
  seek.speed_drpm = m+1;
  reset();
  if (CLAW_COMMISSIONED_MASK == 0) {
    assert(ClawControl_HandleCommand(&seek) == CLAW_RESULT_INVALID); return;
  }
  assert(ClawControl_HandleCommand(&seek) == CLAW_RESULT_OK);
  tick(10,1); assert(motors[m].duty_permille == 0);
  tick(10,1); assert(motors[m].duty_permille == (m == 1 ? 1000 : 900));
  assert(motors[m].command_direction == -1);
  edge(m,-20); tick(10,1); assert(seek_active); /* no 10-count jog cutoff */
  assert(ClawControl_HandleCommand(&seek) == CLAW_RESULT_OK); /* duplicate */
  tick(290,1); assert(seek_active && motors[m].duty_permille == (m == 1 ? 1000 : 900));
  tick(190,1); assert(seek_active);
  tick(10,1); assert(!seek_active && claw_state == CLAW_STATE_IDLE);
  assert(!zero_calibrated && motors[m].position_counts == -20);
  for (unsigned i=0;i<3;++i) assert(motors[i].duty_permille==0);
  reset(); ClawControl_HandleCommand(&seek); tick(20,1); tick(490,1);
  assert(seek_active); tick(10,1);
  assert(claw_state==CLAW_STATE_IDLE && !zero_calibrated); /* no start != proven limit */
  reset(); ClawControl_HandleCommand(&seek); tick(20,1);
  for (unsigned i=0;i<50;++i) { edge(m,-1); tick(100,1); }
  assert(claw_state==CLAW_STATE_FAULT && fault_mask==CLAW_FAULT_SYNC);
  assert(!seek_active && motors[m].duty_permille==0);
  reset(); ClawControl_HandleCommand(&seek); tick(501,0);
  assert(fault_mask==CLAW_FAULT_COMM_TIMEOUT && !seek_active);
  reset(); ClawControl_HandleCommand(&seek); tick(20,1);
  { ClawCommand stop={71,CLAW_COMMAND_STOP,0,0,0}; ClawControl_HandleCommand(&stop); }
  assert(!seek_active && motors[m].duty_permille==0);
  reset(); seek.flags=1; assert(ClawControl_HandleCommand(&seek)==CLAW_RESULT_RANGE);
  puts("Seek: isolation, extended travel, no-progress stop, deadline, watchdog, STOP passed");
}

static void test_group_seek(void)
{
  ClawCommand seek={80,CLAW_COMMAND_SEEK_OPEN,0,0,0};
  reset();
  if (CLAW_COMMISSIONED_MASK != 7) {
    assert(ClawControl_HandleCommand(&seek)==CLAW_RESULT_INVALID); return;
  }
  assert(ClawControl_HandleCommand(&seek)==CLAW_RESULT_OK);
  tick(20,1);
  for (unsigned m=0;m<3;++m)
    assert(motors[m].duty_permille == (m == 1 ? 1000 : 900));
  tick(200,1); edge(1,-10);
  tick(200,1); edge(2,-10);
  tick(100,1);
  assert(seek_active && motors[0].duty_permille==0);
  assert(motors[1].duty_permille==1000 && motors[2].duty_permille==900);
  edge(0,-1); /* residual movement must not restart a stopped channel */
  assert(ClawControl_HandleCommand(&seek)==CLAW_RESULT_OK);
  tick(200,1);
  assert(seek_active && motors[0].duty_permille==0 && motors[1].duty_permille==0);
  assert(motors[2].duty_permille==900);
  tick(200,1);
  assert(!seek_active && claw_state==CLAW_STATE_IDLE && !zero_calibrated);
  for (unsigned m=0;m<3;++m) assert(motors[m].duty_permille==0);
  reset(); ClawControl_HandleCommand(&seek); tick(20,1);
  latch_fault(CLAW_FAULT_MOTOR_2);
  for (unsigned m=0;m<3;++m) assert(motors[m].duty_permille==0);
  assert(!seek_active);
  reset(); ClawControl_HandleCommand(&seek); tick(501,0);
  assert(fault_mask==CLAW_FAULT_COMM_TIMEOUT && !seek_active);
  reset(); ClawControl_HandleCommand(&seek); tick(20,1);
  for (unsigned k=0;k<50;++k) { edge(2,-1); tick(100,1); }
  assert(fault_mask==CLAW_FAULT_SYNC && !seek_active);
  for (unsigned m=0;m<3;++m) assert(motors[m].duty_permille==0);
  reset(); ClawControl_HandleCommand(&seek); tick(20,1);
  { ClawCommand stop={81,CLAW_COMMAND_STOP,0,0,0}; ClawControl_HandleCommand(&stop); }
  assert(!seek_active);
  for (unsigned m=0;m<3;++m) assert(motors[m].duty_permille==0);
  puts("Group seek: simultaneous start, independent latched stops, fault/timeout/STOP passed");
}

static void test_parameters(void)
{
  ClawCommand c={100,CLAW_COMMAND_PARAM_SET,2,105,2}, decoded;
  uint8_t frame[8];
  reset();
  ClawProtocol_EncodeCommand(&c,frame);
  assert(ClawProtocol_DecodeCommand(frame,&decoded) && decoded.opcode==CLAW_COMMAND_PARAM_SET);
  assert(ClawControl_HandleCommand(&c)==CLAW_RESULT_OK);
  assert(profile(1)->ki==10.5f && profile(0)->ki==0 && profile(2)->ki==0);
  ClawControl_BuildParameterFrame(&c,frame);
  assert(frame[1]==100 && frame[2]==2 && frame[3]==2 && frame[4]==105 && frame[6]==0);
  assert(frame[7]==ClawProtocol_Crc8(frame,7));
  c.sequence++; c.flags=6; c.target_cdeg=500; /* below startup: rejected atomically */
  if (CLAW_PROFILE_GROUP==2)
    assert(ClawControl_HandleCommand(&c)==CLAW_RESULT_RANGE && profile(1)->maximum==1000);
  else
    assert(ClawControl_HandleCommand(&c)==CLAW_RESULT_OK && profile(1)->maximum==500);
  c.sequence++; c.flags=2; c.target_cdeg=-1;
  assert(ClawControl_HandleCommand(&c)==CLAW_RESULT_RANGE && profile(1)->ki==10.5f);
  c.sequence++; c.target_cdeg=200; claw_state=CLAW_STATE_MOVING;
  assert(ClawControl_HandleCommand(&c)==CLAW_RESULT_BUSY && profile(1)->ki==10.5f);
  c.sequence++; c.opcode=CLAW_COMMAND_PARAM_GET; c.target_cdeg=0;
  assert(ClawControl_HandleCommand(&c)==CLAW_RESULT_OK);
  c.sequence++; c.speed_drpm=0; c.flags=1;
  assert(ClawControl_HandleCommand(&c)==CLAW_RESULT_INVALID);
  claw_state=CLAW_STATE_IDLE; c.opcode=CLAW_COMMAND_PARAM_SET;
  c.sequence++; c.flags=10; c.target_cdeg=350;
  assert(ClawControl_HandleCommand(&c)==CLAW_RESULT_OK && seek_duty==350);
  c.sequence++; c.flags=11; c.target_cdeg=700;
  assert(ClawControl_HandleCommand(&c)==CLAW_RESULT_OK && seek_no_progress_ms==700);
  c.sequence++; c.flags=12; c.target_cdeg=15000;
  assert(ClawControl_HandleCommand(&c)==CLAW_RESULT_OK && seek_timeout_ms==15000);
  for(unsigned m=0;m<3;++m) assert(motors[m].duty_permille==0);
  assert(!zero_calibrated);
  if(CLAW_COMMISSIONED_MASK==7) {
    c.sequence++; c.opcode=CLAW_COMMAND_SEEK_OPEN; c.flags=0; c.target_cdeg=0;
    assert(ClawControl_HandleCommand(&c)==CLAW_RESULT_OK); tick(20,1);
    assert(motors[0].duty_permille==350); tick(690,1); assert(seek_active);
    tick(10,1); assert(!seek_active);
  }
  reset();
    assert(profile(1)->ki==(CLAW_PROFILE_GROUP==2 ? 20 : 0) &&
           seek_duty==900 && seek_no_progress_ms==500 && seek_timeout_ms==5000);
  puts("RAM parameters: codec, readback, isolated writes, bounds, BUSY, seek effect, reset passed");
}

static void test_grip(void)
{
  ClawCommand c={120,CLAW_COMMAND_GRIP,0,0,0};
  uint8_t data[8]; ClawCommand decoded;
  reset();
  ClawProtocol_EncodeCommand(&c,data);
  assert(ClawProtocol_DecodeCommand(data,&decoded) && decoded.opcode==CLAW_COMMAND_GRIP);
  if(CLAW_COMMISSIONED_MASK!=7) {
    assert(ClawControl_HandleCommand(&c)==CLAW_RESULT_INVALID); return;
  }
  /* Test independent runtime duties, without relying on logical zero or targets. */
  { ClawCommand param={119,CLAW_COMMAND_PARAM_SET,14,400,1};
    assert(ClawControl_HandleCommand(&param)==CLAW_RESULT_OK); }
  grip_duty[1]=350; grip_duty[2]=250;
  for(unsigned m=0;m<3;++m) motors[m].position_counts=2000;
  assert(ClawControl_HandleCommand(&c)==CLAW_RESULT_OK);
  tick(20,1);
  for(unsigned m=0;m<3;++m) {
    assert(motors[m].command_direction==1 && motors[m].duty_permille==600);
    edge(m,2);
  }
  tick(10,1);
  assert(motors[0].duty_permille==400 && motors[1].duty_permille==350 && motors[2].duty_permille==250);
  tick(190,1); edge(1,10); tick(200,1); edge(2,10); tick(100,1);
   assert(grip_active && (motors[0].duty_permille==0 || motors[0].duty_permille==950) &&
          (motors[1].duty_permille==0 || motors[1].duty_permille==350));
  edge(0,1); /* No restart after stop, even with residual movement or repeated command. */
  assert(ClawControl_HandleCommand(&c)==CLAW_RESULT_OK);
   tick(200,1); assert(grip_active && motors[0].duty_permille <= 950 &&
                       motors[1].duty_permille <= 950);
  tick(200,1); assert(grip_active && !seek_active && claw_state==CLAW_STATE_HOLD && !zero_calibrated);
  reset(); ClawControl_HandleCommand(&c); tick(20,1); tick(200,1);
   assert(motors[0].duty_permille==900); /* default grip PWM after startup */
  tick(300,1); assert(grip_active && claw_state==CLAW_STATE_HOLD && fault_mask==0);
  reset(); ClawControl_HandleCommand(&c); tick(20,1);
  for(unsigned k=0;k<50;++k) { edge(0,1); tick(100,1); }
  assert(!grip_active && fault_mask==CLAW_FAULT_SYNC);
  reset(); ClawControl_HandleCommand(&c); tick(501,0);
  assert(!grip_active && fault_mask==CLAW_FAULT_COMM_TIMEOUT);
  reset(); ClawControl_HandleCommand(&c); tick(20,1); ClawControl_TransportFault();
  for(unsigned m=0;m<3;++m) assert(motors[m].duty_permille==0);
  assert(!grip_active);
  reset(); ClawControl_HandleCommand(&c); tick(20,1);
  { ClawCommand stop={121,CLAW_COMMAND_STOP,0,0,0}; ClawControl_HandleCommand(&stop); }
  assert(!grip_active && motors[0].duty_permille==0);
  reset(); c.target_cdeg=1000;
  assert(ClawControl_HandleCommand(&c)==CLAW_RESULT_RANGE);
  reset();
  { ClawCommand param={122,CLAW_COMMAND_PARAM_SET,15,700,0};
    assert(ClawControl_HandleCommand(&param)==CLAW_RESULT_OK);
    param.sequence++; param.flags=16; param.target_cdeg=3000;
    assert(ClawControl_HandleCommand(&param)==CLAW_RESULT_OK);
    param.sequence++; param.flags=14; param.speed_drpm=1; param.target_cdeg=1001;
    assert(ClawControl_HandleCommand(&param)==CLAW_RESULT_RANGE);
  }
  c.sequence=125; c.target_cdeg=0;
  assert(ClawControl_HandleCommand(&c)==CLAW_RESULT_OK);
  tick(20,1); tick(500,1); assert(grip_active);
  tick(200,1); assert(grip_active && claw_state==CLAW_STATE_HOLD && fault_mask==0);
  reset(); assert(grip_duty[0]==900 && grip_no_progress_ms==500 && grip_timeout_ms==5000);
  puts("Grip: no position limit/PID, direction, per-channel PWM, startup, latched stops, deadline/fault passed");
  reset();
}

int main(void)
{
  {
    ClawCommand grip={230,CLAW_COMMAND_GRIP,0,0,0};
    ClawCommand param={231,CLAW_COMMAND_PARAM_SET,18,100,2};
    ClawCommand stop={232,CLAW_COMMAND_STOP,0,0,0};
    ClawCommand seek={233,CLAW_COMMAND_SEEK_OPEN,0,0,0};
    uint8_t frame[8];
    reset();
    assert(hold_duty[0]==950 && hold_duty[1]==950 && hold_duty[2]==950);
    param.target_cdeg=1000;
    assert(parameter_command(&param)==CLAW_RESULT_OK && hold_duty[1]==1000);
    param.target_cdeg=0;
    assert(parameter_command(&param)==CLAW_RESULT_OK && hold_duty[1]==0);
    param.target_cdeg=-1;
    assert(parameter_command(&param)==CLAW_RESULT_RANGE && hold_duty[1]==0);
    param.target_cdeg=100;
    assert(ClawControl_HandleCommand(&param)==CLAW_RESULT_OK && hold_duty[1]==100);
    param.sequence++; param.target_cdeg=1001;
    assert(ClawControl_HandleCommand(&param)==CLAW_RESULT_RANGE && hold_duty[1]==100);
    if(CLAW_COMMISSIONED_MASK==7) {
      hold_duty[0]=80; hold_duty[2]=120;
      assert(ClawControl_HandleCommand(&grip)==CLAW_RESULT_OK); tick(20,1);
      tick(400,1); edge(2,2); tick(100,1);
      assert(claw_state==CLAW_STATE_MOVING && motors[0].duty_permille==80 && motors[1].duty_permille==100);
       assert(motors[2].duty_permille==900);
      tick(400,1); assert(claw_state==CLAW_STATE_HOLD && motors[2].duty_permille==120);
      ClawControl_BuildStatusFrame(frame); assert((frame[7]&0x80)==0);
      for(unsigned k=0;k<60;++k) tick(100,1); /* holds beyond closing deadline */
      assert(grip_active && (motors[0].duty_permille==0 || motors[0].duty_permille==80));
      param.sequence=234; param.target_cdeg=50;
      assert(ClawControl_HandleCommand(&param)==CLAW_RESULT_BUSY);
      assert(ClawControl_HandleCommand(&seek)==CLAW_RESULT_OK && !grip_active);
      for(unsigned m=0;m<3;++m) assert(motors[m].duty_permille==0 && motors[m].command_direction==-1);
      tick(20,1); assert(motors[1].duty_permille==1000);
      ClawControl_HandleCommand(&stop); tick(300,1);
      grip.sequence=235; ClawControl_HandleCommand(&grip); tick(20,1); tick(500,1);
      tick(501,0); assert(fault_mask==CLAW_FAULT_COMM_TIMEOUT && !grip_active);
      for(unsigned m=0;m<3;++m) assert(motors[m].duty_permille==0);
      reset(); hold_duty[1]=100; ClawControl_HandleCommand(&grip); tick(20,1); tick(500,1);
      assert(claw_state==CLAW_STATE_HOLD);
      assert(start_move(0,40,7)==CLAW_RESULT_OK && !grip_active && motors[1].duty_permille==0);
      reset(); hold_duty[1]=100; ClawControl_HandleCommand(&grip); tick(20,1); tick(500,1);
      ClawControl_TransportFault(); assert(!grip_active && motors[1].duty_permille==0);
      puts("PWM hold: independent outputs, watchdog, BUSY, no target bit, seek/open/fault release passed");
    }
  }
  {
    ClawCommand p={200,CLAW_COMMAND_PARAM_SET,10,400,0};
    ClawCommand seek={203,CLAW_COMMAND_SEEK_OPEN,0,0,0};
    reset();
    assert(ClawControl_HandleCommand(&p)==CLAW_RESULT_OK);
    p.sequence++; p.flags=17; p.speed_drpm=2; p.target_cdeg=420;
    assert(ClawControl_HandleCommand(&p)==CLAW_RESULT_OK);
    assert(seek_motor_duty[0]==400 && seek_motor_duty[1]==420 && seek_motor_duty[2]==400);
    p.sequence++; p.target_cdeg=1001;
    assert(ClawControl_HandleCommand(&p)==CLAW_RESULT_RANGE && seek_motor_duty[1]==420);
    if(CLAW_COMMISSIONED_MASK==7) {
      assert(ClawControl_HandleCommand(&seek)==CLAW_RESULT_OK); tick(20,1);
      assert(motors[0].duty_permille==400 && motors[1].duty_permille==420 && motors[2].duty_permille==400);
      p.sequence=204; p.target_cdeg=430;
      assert(ClawControl_HandleCommand(&p)==CLAW_RESULT_BUSY && seek_motor_duty[1]==420);
    }
    reset(); assert(seek_motor_duty[1]==1000);
  }
  test_grip();
  test_parameters();
  test_jog();
  test_seek();
  test_group_seek();
  /* Measured M5 case: 14-count residual error. Integral raises output,
     remains bounded and is cleared for the next action. No plant claim. */
  {
    ClawPidState pi;
    const ClawPidConfig *cfg = profile(1);
    ClawPid_Reset(&pi, 0);
    unsigned first = ClawPid_Step(&pi, cfg, 14, 0, .01f, 1000);
    unsigned last = first;
    for (unsigned n=0; n<79; ++n)
      last = ClawPid_Step(&pi, cfg, 14, 0, .01f, 1000);
    assert(first >= 168 && first < 180);
    assert(last == 318 && pi.integral == 150);
    assert(ClawPid_Step(&pi, cfg, 14, 15, .01f, 1000) == 0);
    ClawPid_Reset(&pi, 0); assert(pi.integral == 0);
  }
#if CLAW_COMMISSIONED_MASK == 0
  reset();
  assert(start_move(100, 40, 1) == CLAW_RESULT_INVALID);
  assert(start_move(100, 40, 7) == CLAW_RESULT_INVALID);
  for (unsigned m=0; m<3; ++m) assert(motors[m].duty_permille == 0);
  tick(10, 0); edge(0, 1); /* passive Hall observation remains possible */
  assert(motors[0].position_counts == 1);
  puts("Uncalibrated production profile: motion disabled, feedback available");
  return 0;
#elif CLAW_COMMISSIONED_MASK == 4
  reset();
  motors[0].position_counts = 1000; /* Restriction must use selected M3 position. */
  assert(start_move(31, 40, 1) == CLAW_RESULT_INVALID);
  assert(start_move(31, 40, 2) == CLAW_RESULT_INVALID);
  assert(start_move(31, 40, 7) == CLAW_RESULT_INVALID);
  assert(start_move(32, 40, 4) == CLAW_RESULT_RANGE);
  assert(start_move(31, 40, 4) == CLAW_RESULT_OK);
  tick(20, 1); assert(motors[2].duty_permille == 600);
  edge(2, 2); tick(10, 1); assert(startup_done[2]);
  for (unsigned k=0; k<200 && claw_state == CLAW_STATE_MOVING; ++k) {
    if (motors[2].duty_permille) edge(2, 1);
    tick(10, 1);
    assert(motors[2].duty_permille <= 1000);
    assert(motors[0].duty_permille == 0 && motors[1].duty_permille == 0);
  }
  assert(claw_state == CLAW_STATE_HOLD && motors[2].duty_permille == 0);
  reset(); start_move(31, 40, 4); tick(20, 1); tick(200, 1);
  assert(claw_state == CLAW_STATE_FAULT && motors[2].duty_permille == 0);
  reset(); start_move(31, 40, 4); tick(501, 0);
  assert(claw_state == CLAW_STATE_FAULT && motors[2].duty_permille == 0);
  puts("M6 trial: selected-channel restriction, endpoint and timeout checks passed");
  return 0;
#elif CLAW_COMMISSIONED_MASK == 2
  reset();
  motors[0].position_counts = 1000; /* Restriction must use selected M2 position. */
  assert(start_move(31, 40, 1) == CLAW_RESULT_INVALID);
  assert(start_move(31, 40, 4) == CLAW_RESULT_INVALID);
  assert(start_move(31, 40, 7) == CLAW_RESULT_INVALID);
  assert(start_move(32, 40, 2) == CLAW_RESULT_RANGE);
  assert(start_move(31, 40, 2) == CLAW_RESULT_OK);
  tick(20, 1); assert(motors[1].duty_permille == 600);
  edge(1, 2); tick(10, 1); assert(startup_done[1]);
  for (unsigned k=0; k<200 && claw_state == CLAW_STATE_MOVING; ++k) {
    if (motors[1].duty_permille) edge(1, 1);
    tick(10, 1);
    assert(motors[1].duty_permille <= 1000);
    assert(motors[0].duty_permille == 0 && motors[2].duty_permille == 0);
  }
  assert(claw_state == CLAW_STATE_HOLD && motors[1].duty_permille == 0);
  reset(); start_move(31, 40, 2); tick(20, 1); tick(200, 1);
  assert(claw_state == CLAW_STATE_FAULT && motors[1].duty_permille == 0);
  reset(); start_move(31, 40, 2); tick(501, 0);
  assert(claw_state == CLAW_STATE_FAULT && motors[1].duty_permille == 0);
  /* After confirmed startup, output may exceed the old 20% ceiling. */
  reset(); assert(start_move(31,40,2) == CLAW_RESULT_OK);
  tick(20,1); edge(1,2); tick(10,1); tick(200,1);
  assert(motors[1].duty_permille > 200 && motors[1].duty_permille <= 498);
  assert(motors[0].duty_permille == 0 && motors[2].duty_permille == 0);
  tick(810,1);
  assert(claw_state == CLAW_STATE_FAULT && motors[1].duty_permille == 0);
  reset(); set_pwm(1,1000); assert(motors[1].duty_permille == 1000);
  { ClawCommand stop = {1, CLAW_COMMAND_STOP, 0, 0, 0};
    ClawControl_HandleCommand(&stop); }
  assert(motors[1].duty_permille == 0);
  puts("M5 trial: range, >20% output, 100% clamp, stop and fault checks passed");
  return 0;
#elif CLAW_COMMISSIONED_MASK == 1
  reset();
  assert(start_move(31, 40, 2) == CLAW_RESULT_INVALID);
  assert(start_move(31, 40, 4) == CLAW_RESULT_INVALID);
  assert(start_move(31, 40, 7) == CLAW_RESULT_INVALID);
  assert(start_move(32, 40, 1) == CLAW_RESULT_RANGE);
  assert(start_move(31, 40, 1) == CLAW_RESULT_OK);
  tick(20, 1); assert(motors[0].duty_permille == 600);
  edge(0, 2); tick(10, 1); assert(startup_done[0]);
  for (unsigned k=0; k<200 && claw_state == CLAW_STATE_MOVING; ++k) {
    if (motors[0].duty_permille) edge(0, 1);
    tick(10, 1);
    assert(motors[0].duty_permille <= 1000);
    assert(motors[1].duty_permille == 0 && motors[2].duty_permille == 0);
  }
  assert(claw_state == CLAW_STATE_HOLD && motors[0].duty_permille == 0);
  reset(); start_move(31, 40, 1); tick(20, 1); tick(200, 1);
  assert(claw_state == CLAW_STATE_FAULT && motors[0].duty_permille == 0);
  reset(); start_move(31, 40, 1); tick(501, 0);
  assert(claw_state == CLAW_STATE_FAULT && motors[0].duty_permille == 0);
  puts("M4 trial: channel/step restriction, startup, endpoint and timeout checks passed");
  return 0;
#elif CLAW_TRIAL_MAX_STEP_MDEG > 0
  const int32_t limit = ClawControl_AngleMilliDegToCounts(CLAW_TRIAL_MAX_STEP_MDEG);
  reset();
  /* Every selected channel must satisfy the bound, not just the mean. */
  motors[2].position_counts = -1;
  assert(start_move(limit, 40, 7) == CLAW_RESULT_RANGE);
  reset(); motors[1].position_counts = 2 * limit + 1;
  assert(start_move(limit, 40, 7) == CLAW_RESULT_RANGE);
  reset(); assert(start_move(limit + 1, 40, 7) == CLAW_RESULT_RANGE);
  assert(start_move(limit, 40, 7) == CLAW_RESULT_OK);
  tick(20, 1);
  for (unsigned m=0; m<3; ++m) assert(motors[m].duty_permille == profile(m)->startup);
  for (unsigned k=0; k<150 && claw_state == CLAW_STATE_MOVING; ++k) {
    for (unsigned m=0; m<3; ++m) {
      assert(motors[m].duty_permille <= profile(m)->maximum);
      if (motors[m].duty_permille) edge(m, 1);
    }
    tick(10, 1);
  }
  assert(claw_state == CLAW_STATE_HOLD && fault_mask == 0);
  for (unsigned m=0; m<3; ++m) assert(motors[m].duty_permille == 0);
  reset(); start_move(limit,40,7); tick(20,1); tick(200,1);
  assert(claw_state == CLAW_STATE_FAULT);
  for (unsigned m=0; m<3; ++m) assert(motors[m].duty_permille == 0);
  reset(); start_move(limit,40,7); tick(501,0);
  assert(fault_mask == CLAW_FAULT_COMM_TIMEOUT);
  puts("Bounded group: per-channel range, startup, arrival and fault stop passed");
  return 0;
#else
  unsigned i, k;
  /* Production defaults: no temporary step limit, 60% startup on all channels. */
  reset();
  { ClawCommand close = {9, CLAW_COMMAND_SET_TARGET, 0, 2000, 40};
    assert(ClawControl_HandleCommand(&close) == CLAW_RESULT_OK); }
  tick(20,1);
  for (i=0; i<3; ++i) assert(motors[i].duty_permille == 600);
  for (i=0; i<3; ++i) edge(i,2);
  tick(10,1);
  for (i=0; i<3; ++i) assert(startup_done[i]);
  /* Behind a moving reference: PID can use >20%, then physical 100% maximum. */
  tick(500,1);
  for (i=0; i<3; ++i) assert(motors[i].duty_permille == 1000);
  tick(310,1);
  assert(claw_state == CLAW_STATE_FAULT);
  for (i=0; i<3; ++i) assert(motors[i].duty_permille == 0);
  reset();
  { ClawCommand invalid = {10, CLAW_COMMAND_SET_TARGET, 0, 12001, 40};
    assert(ClawControl_HandleCommand(&invalid) == CLAW_RESULT_RANGE); }
  set_pwm(0,1100); assert(motors[0].duty_permille == 1000);
  reset();
  ClawPidConfig p = {12, 10, 1, 150, 200, 700, 618, 2, 400};
  ClawPidState a, b;
  ClawPid_Reset(&a, 0); ClawPid_Reset(&b, 0);
  assert(ClawPid_Step(&a, &p, 40, 0, .01f, 700) >
         ClawPid_Step(&b, &p, 40, 20, .01f, 700));
  for (i=0; i<1000; ++i) ClawPid_Step(&a, &p, 1000, 0, .01f, 700);
  assert(a.integral <= 150 && a.integral >= -150);
  assert(ClawPid_Step(&a, &p, 10, 20, .01f, 700) == 0);
  assert(ClawPid_Step(&a, &p, 1000, 20, .01f, 200) <= 200);

  reset(); assert(start_move(100, 40, 1) == CLAW_RESULT_OK);
  tick(10, 1); assert(motors[0].duty_permille == 0);
  tick(10, 1); assert(motors[0].duty_permille == 600);
  edge(0, 2); tick(10, 1); assert(startup_done[0]);
  assert(motors[1].duty_permille == 0 && motors[2].duty_permille == 0);
  tick(810, 1); assert(fault_mask == CLAW_FAULT_MOTOR_1);

  reset(); start_move(100, 40, 1); tick(20, 1); tick(400, 1);
  assert(claw_state == CLAW_STATE_FAULT && fault_mask == 1);
  assert(motors[0].duty_permille == 0);
  reset(); start_move(100, 40, 7); tick(501, 0);
  assert(fault_mask == CLAW_FAULT_COMM_TIMEOUT);
  reset(); start_move(100, 40, 1); tick(15001, 1);
  assert(fault_mask == CLAW_FAULT_SYNC);
  reset(); start_move(1000, 40, 7); tick(20, 1);
  for(i=0; i<3; ++i) { startup_done[i] = true; motors[i].last_hall_tick=test_tick; }
  edge(2, 200); tick(10, 1); tick(2010, 1);
  assert(fault_mask == CLAW_FAULT_SYNC);

  reset(); start_move(1000, 40, 1); tick(20, 1);
  edge(0, 500); tick(10, 1); /* ahead: deliberate zero output */
  assert(motors[0].duty_permille == 0);
  tick(810, 1); assert(claw_state == CLAW_STATE_MOVING && fault_mask == 0);

  reset(); start_move(0, 40, 7); tick(20, 1);
  assert(claw_state == CLAW_STATE_HOLD);
  reset(); start_move(2, 40, 1); tick(20, 1); edge(0, 1); tick(10, 1);
  assert(claw_state == CLAW_STATE_HOLD); /* endpoint wins over startup */

  reset(); motors[0].position_counts=200; motors[2].position_counts=100;
  start_move(100, 40, 7);
  assert(motors[0].command_direction == -1 && motors[1].command_direction == 1);
  tick(20, 1); assert(motors[2].duty_permille == 0);
  edge(0, -101); edge(1, 101); tick(10, 1);
  assert(claw_state == CLAW_STATE_HOLD);
  for(i=0; i<3; ++i) assert(motors[i].duty_permille == 0);

  reset(); start_move(100, 40, 1); tick(20, 1);
  { ClawCommand stop = {1, CLAW_COMMAND_STOP, 0, 0, 0};
    ClawControl_HandleCommand(&stop); }
  assert(claw_state == CLAW_STATE_IDLE && motors[0].duty_permille == 0);
  latch_fault(CLAW_FAULT_SYNC);
  tick(CLAW_DIRECTION_QUIET_MS, 1);
  { ClawCommand single = {2, CLAW_COMMAND_SET_TARGET_M1, 0, 500, 40};
    assert(ClawControl_HandleCommand(&single) == CLAW_RESULT_OK); }
  latch_fault(CLAW_FAULT_MOTOR_1);
  { ClawCommand single = {3, CLAW_COMMAND_SET_TARGET_M1, 0, 500, 40};
    assert(ClawControl_HandleCommand(&single) == CLAW_RESULT_FAULT); }

  /* Unequal idealized motor gains, same trajectory, actual C controller.
     This synthetic plant is not a measured mechanical model. */
  reset(); start_move(300, 40, 7);
  {
    float residual[3] = {0,0,0};
    const float gain[3] = {3.0f, 4.0f, 5.0f};
    unsigned arrival[3] = {0,0,0};
    for(k=0; k<1500 && claw_state == CLAW_STATE_MOVING; ++k) {
      for(i=0; i<3; ++i) {
        int delta;
        assert(motors[i].duty_permille <= profile(i)->maximum);
        residual[i] += motors[i].duty_permille * gain[i] * .01f;
        delta = (int)residual[i]; residual[i] -= delta;
        if(delta) edge(i, delta);
        if(!arrival[i] && motors[i].position_counts >= 299) arrival[i] = k*10;
      }
      tick(10, 1);
    }
    assert(claw_state == CLAW_STATE_HOLD && fault_mask == 0);
    printf("Synthetic arrivals ms: %u %u %u\n",arrival[0],arrival[1],arrival[2]);
  }
  puts("Native controller checks passed");
  return 0;
#endif
}
