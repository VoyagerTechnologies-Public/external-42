#include "42.h"
#include "shire_ipc_protocol.h"
#include <errno.h>
#include <math.h>
#define EXTERN extern
#include "Ac.h"
#undef EXTERN

static int ShireBinaryIpcEnabled(void)
{
      const char *Mode = getenv("FORTYTWO_IPC_MODE");
      return Mode == NULL || strcmp(Mode,"text");
}

static int ShireWriteAll(SOCKET Socket, const void *Buffer, size_t Length)
{
      const char *Cursor = Buffer;
      while(Length > 0) {
         ssize_t Count = write(Socket,Cursor,Length);
         if (Count < 0 && errno == EINTR) continue;
         if (Count <= 0) return -1;
         Cursor += Count;
         Length -= (size_t)Count;
      }
      return 0;
}

static int ShireReadAll(SOCKET Socket, void *Buffer, size_t Length)
{
      char *Cursor = Buffer;
      while(Length > 0) {
         ssize_t Count = read(Socket,Cursor,Length);
         if (Count < 0 && errno == EINTR) continue;
         if (Count <= 0) return -1;
         Cursor += Count;
         Length -= (size_t)Count;
      }
      return 0;
}

static int ShireReadTextFrame(SOCKET Socket, char *Buffer, size_t Capacity,
   size_t *Length)
{
      static const char EndMarker[] = "[ENDMSG]\n";
      size_t Used = 0;

      if (Buffer == NULL || Length == NULL || Capacity < sizeof(EndMarker))
         return -1;
      while(Used < Capacity-1) {
         ssize_t Count = read(Socket,&Buffer[Used],Capacity-1-Used);
         if (Count < 0 && errno == EINTR) continue;
         if (Count <= 0) return -1;
         Used += (size_t)Count;
         Buffer[Used] = '\0';
         if (strstr(Buffer,EndMarker) != NULL) {
            *Length = Used;
            return 0;
         }
      }
      return -1;
}

static void ShireWriteBinaryState(SOCKET Socket)
{
      shire_ipc_state_t State;
      long i;
      memset(&State,0,sizeof(State));
      State.header.magic = SHIRE_IPC_MAGIC;
      State.header.version = SHIRE_IPC_VERSION;
      State.header.type = SHIRE_IPC_STATE;
      State.header.payload_size = sizeof(State)-sizeof(State.header);
      State.sim_time = SimTime;
      State.utc_civil_time = CivilTime;
      if (Nsc > 0 && SC[0].Exists) {
         for(i=0;i<4;i++) State.qn[i] = SC[0].qn[i];
         for(i=0;i<3;i++) {
            State.wn[i] = SC[0].wn[i];
            State.sun_vector_body[i] = SC[0].svb[i];
            State.mag_field_body[i] = SC[0].bvb[i];
            State.hvb[i] = SC[0].Hvb[i];
            State.cm[i] = SC[0].cm[i];
            State.inertia[i][0] = SC[0].I[i][0];
            State.inertia[i][1] = SC[0].I[i][1];
            State.inertia[i][2] = SC[0].I[i][2];
         }
         State.mass = SC[0].mass;
         State.eclipse = (int32_t)SC[0].Eclipse;
         State.atmo_density = SC[0].AtmoDensity;
      }
      if (Norb > 0 && Orb[0].Exists) {
         for(i=0;i<3;i++) {
            State.pos_n[i] = Orb[0].PosN[i];
            State.vel_n[i] = Orb[0].VelN[i];
         }
      }
      if (ShireWriteAll(Socket,&State,sizeof(State)) != 0) {
         printf("Error writing binary state to socket.\n");
         exit(1);
      }
}

static void ShireReadBinaryCommands(SOCKET Socket)
{
      shire_ipc_commands_t Batch;
      shire_ipc_ack_t Ack;
      uint32_t c,i;
      int Valid = 1;
      memset(&Batch,0,sizeof(Batch));
      if (ShireReadAll(Socket,&Batch.header,sizeof(Batch.header)) != 0) {
         printf("Error reading binary commands from socket.\n");
         exit(1);
      }
      if (Batch.header.payload_size > sizeof(Batch)-sizeof(Batch.header)) {
         printf("Oversized SHIRE binary command frame.\n");
         exit(1);
      }
      if (ShireReadAll(Socket,&Batch.count,Batch.header.payload_size) != 0) {
         printf("Error reading binary command payload from socket.\n");
         exit(1);
      }
      memset(&Ack,0,sizeof(Ack));
      Ack.header.magic = SHIRE_IPC_MAGIC;
      Ack.header.version = SHIRE_IPC_VERSION;
      Ack.header.type = SHIRE_IPC_ACK;
      Ack.header.payload_size = sizeof(Ack)-sizeof(Ack.header);
      if (Batch.header.magic != SHIRE_IPC_MAGIC ||
          Batch.header.version != SHIRE_IPC_VERSION ||
          Batch.header.type != SHIRE_IPC_COMMANDS ||
          Batch.count > SHIRE_IPC_MAX_COMMANDS ||
          Batch.header.payload_size !=
             SHIRE_IPC_COMMANDS_PAYLOAD_SIZE(Batch.count)) {
         printf("Invalid SHIRE binary command frame.\n");
         Valid = 0;
      }
      for(c=0;Valid && c<Batch.count;c++) {
         shire_ipc_command_t *Cmd = &Batch.commands[c];
         if (Cmd->spacecraft_id < 0 || Cmd->spacecraft_id >= Nsc ||
             !SC[Cmd->spacecraft_id].Exists ||
             (Cmd->type != SHIRE_IPC_CMD_WHEEL &&
              Cmd->type != SHIRE_IPC_CMD_MTB &&
              Cmd->type != SHIRE_IPC_CMD_THRUSTER)) {
            printf("Invalid SHIRE binary command target/type.\n");
            Valid = 0;
         }
         for(i=0;Valid && i<3;i++) {
            if ((Cmd->enable_mask & (1U << i)) != 0 &&
                (!isfinite(Cmd->values[i]) ||
                 (Cmd->type == SHIRE_IPC_CMD_THRUSTER &&
                  !isfinite(Cmd->values[i+3])))) {
               printf("Invalid non-finite SHIRE binary command value.\n");
               Valid = 0;
            }
         }
         if (Valid && Cmd->type == SHIRE_IPC_CMD_WHEEL &&
             (Cmd->enable_mask & (1U << 3)) != 0 &&
             !isfinite(Cmd->values[3])) {
            printf("Invalid non-finite SHIRE wheel command value.\n");
            Valid = 0;
         }
      }
      for(c=0;Valid && c<Batch.count;c++) {
         shire_ipc_command_t *Cmd = &Batch.commands[c];
         if (Cmd->type == SHIRE_IPC_CMD_WHEEL) {
            for(i=0;i<4 && i<(uint32_t)SC[Cmd->spacecraft_id].Nw;i++)
               if (Cmd->enable_mask & (1U << i))
                  SC[Cmd->spacecraft_id].Whl[i].Tcmd = Cmd->values[i];
         }
         else if (Cmd->type == SHIRE_IPC_CMD_MTB) {
            for(i=0;i<3 && i<(uint32_t)SC[Cmd->spacecraft_id].Nmtb;i++)
               if (Cmd->enable_mask & (1U << i))
                  SC[Cmd->spacecraft_id].MTB[i].Mcmd = Cmd->values[i];
         }
         else if (Cmd->type == SHIRE_IPC_CMD_THRUSTER) {
            for(i=0;i<3;i++) {
               if (Cmd->enable_mask & (1U << i)) {
                  SC[Cmd->spacecraft_id].IdealAct[i].Fcmd = Cmd->values[i];
                  SC[Cmd->spacecraft_id].IdealAct[i].Tcmd = Cmd->values[i+3];
               }
            }
         }
      }
      Ack.status = Valid ? 0 : -1;
      if (ShireWriteAll(Socket,&Ack,sizeof(Ack)) != 0) {
         printf("Error acknowledging binary commands.\n");
         exit(1);
      }
}

/******************************************************************************/
void WriteToSocket(SOCKET Socket,  char **Prefix, long Nprefix, long EchoEnabled)
{
      if (ShireBinaryIpcEnabled()) {
         ShireWriteBinaryState(Socket);
         return;
      }
      struct SCType *S;
      struct WorldType *W;
      struct OrbitType *O;
      struct CommLinkType *L;
      int Success;
      char Ack[4] = "Ack\0";
      long Is,Ipfx;
      char Msg[16384];
      long MsgLen = 0;
      long LineLen,PfxLen;
      char line[512];
      long k;

      sprintf(line,"TIME %ld-%03ld-%02ld:%02ld:%012.9lf\n",
         UTC.Year,UTC.doy,UTC.Hour,UTC.Minute,UTC.Second);
      LineLen = strlen(line);
      memcpy(&Msg[MsgLen],line,LineLen);
      MsgLen += LineLen;
      if (EchoEnabled) printf("%s",line);

      for(Ipfx=0;Ipfx<Nprefix;Ipfx++) {
         PfxLen = strlen(Prefix[Ipfx]);

         for(Is=0;Is<Nsc;Is++) {
            if (SC[Is].Exists) {
               S = &SC[Is];

               sprintf(line,"SC[%ld].qn = [%18.12le %18.12le %18.12le %18.12le]\n",Is,
                  S->qn[0],S->qn[1],S->qn[2],S->qn[3]);
               if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                  LineLen = strlen(line);
                  memcpy(&Msg[MsgLen],line,LineLen);
                  MsgLen += LineLen;
                  if (EchoEnabled) printf("%s",line);
               }

               sprintf(line,"SC[%ld].wn = [%18.12le %18.12le %18.12le]\n",Is,
                  S->wn[0],S->wn[1],S->wn[2]);
               if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                  LineLen = strlen(line);
                  memcpy(&Msg[MsgLen],line,LineLen);
                  MsgLen += LineLen;
                  if (EchoEnabled) printf("%s",line);
               }

               sprintf(line,"SC[%ld].PosR = [%18.12le %18.12le %18.12le]\n",Is,
                  S->PosR[0],S->PosR[1],S->PosR[2]);
               if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                  LineLen = strlen(line);
                  memcpy(&Msg[MsgLen],line,LineLen);
                  MsgLen += LineLen;
                  if (EchoEnabled) printf("%s",line);
               }

               sprintf(line,"SC[%ld].VelR = [%18.12le %18.12le %18.12le]\n",Is,
                  S->VelR[0],S->VelR[1],S->VelR[2]);
               if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                  LineLen = strlen(line);
                  memcpy(&Msg[MsgLen],line,LineLen);
                  MsgLen += LineLen;
                  if (EchoEnabled) printf("%s",line);
               }

               sprintf(line,"SC[%ld].svb = [%18.12le %18.12le %18.12le]\n",Is,
                  S->svb[0],S->svb[1],S->svb[2]);
               if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                  LineLen = strlen(line);
                  memcpy(&Msg[MsgLen],line,LineLen);
                  MsgLen += LineLen;
                  if (EchoEnabled) printf("%s",line);
               }

               sprintf(line,"SC[%ld].bvb = [%18.12le %18.12le %18.12le]\n",Is,
                  S->bvb[0],S->bvb[1],S->bvb[2]);
               if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                  LineLen = strlen(line);
                  memcpy(&Msg[MsgLen],line,LineLen);
                  MsgLen += LineLen;
                  if (EchoEnabled) printf("%s",line);
               }

               sprintf(line,"SC[%ld].Hvb = [%18.12le %18.12le %18.12le]\n",Is,
                  S->Hvb[0],S->Hvb[1],S->Hvb[2]);
               if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                  LineLen = strlen(line);
                  memcpy(&Msg[MsgLen],line,LineLen);
                  MsgLen += LineLen;
                  if (EchoEnabled) printf("%s",line);
               }

               for(k=0;k<S->Nb;k++) {
                  sprintf(line,"SC[%ld].B[%ld].wn = [%18.12le %18.12le %18.12le]\n",Is,k,
                     S->B[k].wn[0],S->B[k].wn[1],S->B[k].wn[2]);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Nb;k++) {
                  sprintf(line,"SC[%ld].B[%ld].qn = [%18.12le %18.12le %18.12le %18.12le]\n",Is,k,
                     S->B[k].qn[0],S->B[k].qn[1],S->B[k].qn[2],S->B[k].qn[3]);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Ng;k++) {
                  sprintf(line,"SC[%ld].G[%ld].Pos = [%18.12le %18.12le %18.12le]\n",Is,k,
                     S->G[k].Pos[0],S->G[k].Pos[1],S->G[k].Pos[2]);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Ng;k++) {
                  sprintf(line,"SC[%ld].G[%ld].PosRate = [%18.12le %18.12le %18.12le]\n",Is,k,
                     S->G[k].PosRate[0],S->G[k].PosRate[1],S->G[k].PosRate[2]);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Ng;k++) {
                  sprintf(line,"SC[%ld].G[%ld].Ang = [%18.12le %18.12le %18.12le]\n",Is,k,
                     S->G[k].Ang[0],S->G[k].Ang[1],S->G[k].Ang[2]);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Ng;k++) {
                  sprintf(line,"SC[%ld].G[%ld].AngRate = [%18.12le %18.12le %18.12le]\n",Is,k,
                     S->G[k].AngRate[0],S->G[k].AngRate[1],S->G[k].AngRate[2]);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Nw;k++) {
                  sprintf(line,"SC[%ld].Whl[%ld].H = %18.12le\n",Is,k,
                     S->Whl[k].H);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Ngyro;k++) {
                  sprintf(line,"SC[%ld].Gyro[%ld].TrueRate = %18.12le\n",Is,k,
                     S->Gyro[k].TrueRate);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Nmag;k++) {
                  sprintf(line,"SC[%ld].MAG[%ld].Field = %18.12le\n",Is,k,
                     S->MAG[k].Field);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Ncss;k++) {
                  sprintf(line,"SC[%ld].CSS[%ld].Valid = %ld\n",Is,k,
                     S->CSS[k].Valid);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Ncss;k++) {
                  sprintf(line,"SC[%ld].CSS[%ld].Illum = %18.12le\n",Is,k,
                     S->CSS[k].Illum);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Nfss;k++) {
                  sprintf(line,"SC[%ld].FSS[%ld].Valid = %ld\n",Is,k,
                     S->FSS[k].Valid);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Nfss;k++) {
                  sprintf(line,"SC[%ld].FSS[%ld].SunAng = [%18.12le %18.12le]\n",Is,k,
                     S->FSS[k].SunAng[0],S->FSS[k].SunAng[1]);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Nst;k++) {
                  sprintf(line,"SC[%ld].ST[%ld].Valid = %ld\n",Is,k,
                     S->ST[k].Valid);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Nst;k++) {
                  sprintf(line,"SC[%ld].ST[%ld].qn = [%18.12le %18.12le %18.12le %18.12le]\n",Is,k,
                     S->ST[k].qn[0],S->ST[k].qn[1],S->ST[k].qn[2],S->ST[k].qn[3]);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Ngps;k++) {
                  sprintf(line,"SC[%ld].GPS[%ld].Valid = %ld\n",Is,k,
                     S->GPS[k].Valid);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Ngps;k++) {
                  sprintf(line,"SC[%ld].GPS[%ld].Rollover = %ld\n",Is,k,
                     S->GPS[k].Rollover);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Ngps;k++) {
                  sprintf(line,"SC[%ld].GPS[%ld].Week = %ld\n",Is,k,
                     S->GPS[k].Week);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Ngps;k++) {
                  sprintf(line,"SC[%ld].GPS[%ld].Sec = %18.12le\n",Is,k,
                     S->GPS[k].Sec);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Ngps;k++) {
                  sprintf(line,"SC[%ld].GPS[%ld].PosN = [%18.12le %18.12le %18.12le]\n",Is,k,
                     S->GPS[k].PosN[0],S->GPS[k].PosN[1],S->GPS[k].PosN[2]);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Ngps;k++) {
                  sprintf(line,"SC[%ld].GPS[%ld].VelN = [%18.12le %18.12le %18.12le]\n",Is,k,
                     S->GPS[k].VelN[0],S->GPS[k].VelN[1],S->GPS[k].VelN[2]);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Ngps;k++) {
                  sprintf(line,"SC[%ld].GPS[%ld].PosW = [%18.12le %18.12le %18.12le]\n",Is,k,
                     S->GPS[k].PosW[0],S->GPS[k].PosW[1],S->GPS[k].PosW[2]);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Ngps;k++) {
                  sprintf(line,"SC[%ld].GPS[%ld].VelW = [%18.12le %18.12le %18.12le]\n",Is,k,
                     S->GPS[k].VelW[0],S->GPS[k].VelW[1],S->GPS[k].VelW[2]);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }

               for(k=0;k<S->Nacc;k++) {
                  sprintf(line,"SC[%ld].Accel[%ld].TrueAcc = %18.12le\n",Is,k,
                     S->Accel[k].TrueAcc);
                  if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                     LineLen = strlen(line);
                     memcpy(&Msg[MsgLen],line,LineLen);
                     MsgLen += LineLen;
                     if (EchoEnabled) printf("%s",line);
                  }
               }
            }
         }

         for(Is=0;Is<NWORLD;Is++) {
            if (World[Is].Exists) {
               W = &World[Is];

               sprintf(line,"World[%ld].PosH = [%18.12le %18.12le %18.12le]\n",Is,
                  W->PosH[0],W->PosH[1],W->PosH[2]);
               if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                  LineLen = strlen(line);
                  memcpy(&Msg[MsgLen],line,LineLen);
                  MsgLen += LineLen;
                  if (EchoEnabled) printf("%s",line);
               }

               sprintf(line,"World[%ld].eph.PosN = [%18.12le %18.12le %18.12le]\n",Is,
                  W->eph.PosN[0],W->eph.PosN[1],W->eph.PosN[2]);
               if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                  LineLen = strlen(line);
                  memcpy(&Msg[MsgLen],line,LineLen);
                  MsgLen += LineLen;
                  if (EchoEnabled) printf("%s",line);
               }

               sprintf(line,"World[%ld].eph.VelN = [%18.12le %18.12le %18.12le]\n",Is,
                  W->eph.VelN[0],W->eph.VelN[1],W->eph.VelN[2]);
               if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                  LineLen = strlen(line);
                  memcpy(&Msg[MsgLen],line,LineLen);
                  MsgLen += LineLen;
                  if (EchoEnabled) printf("%s",line);
               }
            }
         }

         for(Is=0;Is<Norb;Is++) {
            if (Orb[Is].Exists) {
               O = &Orb[Is];

               sprintf(line,"Orb[%ld].PosN = [%18.12le %18.12le %18.12le]\n",Is,
                  O->PosN[0],O->PosN[1],O->PosN[2]);
               if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                  LineLen = strlen(line);
                  memcpy(&Msg[MsgLen],line,LineLen);
                  MsgLen += LineLen;
                  if (EchoEnabled) printf("%s",line);
               }

               sprintf(line,"Orb[%ld].VelN = [%18.12le %18.12le %18.12le]\n",Is,
                  O->VelN[0],O->VelN[1],O->VelN[2]);
               if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                  LineLen = strlen(line);
                  memcpy(&Msg[MsgLen],line,LineLen);
                  MsgLen += LineLen;
                  if (EchoEnabled) printf("%s",line);
               }
            }
         }

         for(Is=0;Is<Nlink;Is++) {
            if (CommLink[Is].Exists) {
               L = &CommLink[Is];

               sprintf(line,"CommLink[%ld].Doppler = %18.12le\n",Is,
                  L->Doppler);
               if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                  LineLen = strlen(line);
                  memcpy(&Msg[MsgLen],line,LineLen);
                  MsgLen += LineLen;
                  if (EchoEnabled) printf("%s",line);
               }

               sprintf(line,"CommLink[%ld].Delay = %18.12le\n",Is,
                  L->Delay);
               if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                  LineLen = strlen(line);
                  memcpy(&Msg[MsgLen],line,LineLen);
                  MsgLen += LineLen;
                  if (EchoEnabled) printf("%s",line);
               }

               sprintf(line,"CommLink[%ld].Carrier = %18.12le\n",Is,
                  L->Carrier);
               if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                  LineLen = strlen(line);
                  memcpy(&Msg[MsgLen],line,LineLen);
                  MsgLen += LineLen;
                  if (EchoEnabled) printf("%s",line);
               }

               sprintf(line,"CommLink[%ld].CNR = %18.12le\n",Is,
                  L->CNR);
               if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                  LineLen = strlen(line);
                  memcpy(&Msg[MsgLen],line,LineLen);
                  MsgLen += LineLen;
                  if (EchoEnabled) printf("%s",line);
               }

               sprintf(line,"CommLink[%ld].Range = %18.12le\n",Is,
                  L->Range);
               if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                  LineLen = strlen(line);
                  memcpy(&Msg[MsgLen],line,LineLen);
                  MsgLen += LineLen;
                  if (EchoEnabled) printf("%s",line);
               }

               sprintf(line,"CommLink[%ld].RangeRate = %18.12le\n",Is,
                  L->RangeRate);
               if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                  LineLen = strlen(line);
                  memcpy(&Msg[MsgLen],line,LineLen);
                  MsgLen += LineLen;
                  if (EchoEnabled) printf("%s",line);
               }

               sprintf(line,"CommLink[%ld].PathIsOcculted = %ld\n",Is,
                  L->PathIsOcculted);
               if (!strncmp(line,Prefix[Ipfx],PfxLen)) {
                  LineLen = strlen(line);
                  memcpy(&Msg[MsgLen],line,LineLen);
                  MsgLen += LineLen;
                  if (EchoEnabled) printf("%s",line);
               }
            }
         }

      }

      sprintf(line,"[ENDMSG]\n");
      LineLen = strlen(line);
      memcpy(&Msg[MsgLen],line,LineLen);
      MsgLen += LineLen;
      if (EchoEnabled) printf("%s",line);
      if (EchoEnabled) printf("MsgLen = %ld\n",MsgLen);
      printf("\n");

      Success = ShireWriteAll(Socket,Msg,(size_t)MsgLen);
      if (Success != 0) {
         printf("Error writing to socket in WriteToSocket.\n");
         exit(1);
      }
      if (ShireReadAll(Socket,Ack,sizeof(Ack)) != 0 ||
          memcmp(Ack,"Ack",sizeof(Ack)) != 0) {
         printf("Error reading acknowledgement in WriteToSocket.\n");
         exit(1);
      }

}
/******************************************************************************/
void ReadFromSocket(SOCKET Socket, long EchoEnabled)
{
      if (ShireBinaryIpcEnabled()) {
         ShireReadBinaryCommands(Socket);
         return;
      }

      struct SCType *S;
      struct OrbitType *O;
      struct DynType *D;
      long Is,i;
      char line[512] = "Blank";
      long RequestTimeRefresh = 0;
      long Done;
      char Msg[32768];
      long Imsg,Iline;
      size_t NumBytes;
      double DbleVal[30];
      long LongVal[30];
      long Year,doy,Hour,Minute;
      double Second;
      char Ack[4] = "Ack\0";
      long k;

      if (ShireReadTextFrame(Socket,Msg,sizeof(Msg),&NumBytes) != 0) {
         printf("Error reading complete text frame from socket in ReadFromSocket.\n");
         exit(1);
      }

      Done = 0;
      Imsg = 0;
      while(!Done) {
         /* Parse lines from Msg, newline-delimited */
         Iline = 0;
         memset(line,'\0',512);
         while(((size_t)Imsg < NumBytes) && (Msg[Imsg] != '\n') && (Iline < 511)) {
            line[Iline++] = Msg[Imsg++];
         }
         if ((size_t)Imsg >= NumBytes || Msg[Imsg] != '\n') {
            printf("Malformed text frame in ReadFromSocket.\n");
            exit(1);
         }
         line[Iline++] = Msg[Imsg++];
         if (EchoEnabled) printf("%s",line);

         if (sscanf(line,"TIME %ld-%ld-%ld:%ld:%lf",
            &Year,&doy,&Hour,&Minute,&Second) == 5) {
            RequestTimeRefresh = 1;
         }


      if (sscanf(line,"SC[%ld].qn = [%le %le %le %le]",
         &Is,
         &DbleVal[0],
         &DbleVal[1],
         &DbleVal[2],
         &DbleVal[3]) == 5)
      {
         SC[Is].qn[0] = DbleVal[0];
         SC[Is].qn[1] = DbleVal[1];
         SC[Is].qn[2] = DbleVal[2];
         SC[Is].qn[3] = DbleVal[3];
         SC[Is].RequestStateRefresh = 1;
      }

      if (sscanf(line,"SC[%ld].wn = [%le %le %le]",
         &Is,
         &DbleVal[0],
         &DbleVal[1],
         &DbleVal[2]) == 4)
      {
         SC[Is].wn[0] = DbleVal[0];
         SC[Is].wn[1] = DbleVal[1];
         SC[Is].wn[2] = DbleVal[2];
         SC[Is].RequestStateRefresh = 1;
      }

      if (sscanf(line,"SC[%ld].PosR = [%le %le %le]",
         &Is,
         &DbleVal[0],
         &DbleVal[1],
         &DbleVal[2]) == 4)
      {
         SC[Is].PosR[0] = DbleVal[0];
         SC[Is].PosR[1] = DbleVal[1];
         SC[Is].PosR[2] = DbleVal[2];
         SC[Is].RequestStateRefresh = 1;
      }

      if (sscanf(line,"SC[%ld].VelR = [%le %le %le]",
         &Is,
         &DbleVal[0],
         &DbleVal[1],
         &DbleVal[2]) == 4)
      {
         SC[Is].VelR[0] = DbleVal[0];
         SC[Is].VelR[1] = DbleVal[1];
         SC[Is].VelR[2] = DbleVal[2];
         SC[Is].RequestStateRefresh = 1;
      }

      if (sscanf(line,"SC[%ld].svb = [%le %le %le]",
         &Is,
         &DbleVal[0],
         &DbleVal[1],
         &DbleVal[2]) == 4)
      {
         SC[Is].svb[0] = DbleVal[0];
         SC[Is].svb[1] = DbleVal[1];
         SC[Is].svb[2] = DbleVal[2];
         SC[Is].RequestStateRefresh = 1;
      }

      if (sscanf(line,"SC[%ld].bvb = [%le %le %le]",
         &Is,
         &DbleVal[0],
         &DbleVal[1],
         &DbleVal[2]) == 4)
      {
         SC[Is].bvb[0] = DbleVal[0];
         SC[Is].bvb[1] = DbleVal[1];
         SC[Is].bvb[2] = DbleVal[2];
         SC[Is].RequestStateRefresh = 1;
      }

      if (sscanf(line,"SC[%ld].Hvb = [%le %le %le]",
         &Is,
         &DbleVal[0],
         &DbleVal[1],
         &DbleVal[2]) == 4)
      {
         SC[Is].Hvb[0] = DbleVal[0];
         SC[Is].Hvb[1] = DbleVal[1];
         SC[Is].Hvb[2] = DbleVal[2];
         SC[Is].RequestStateRefresh = 1;
      }

      if (sscanf(line,"SC[%ld].B[%ld].wn = [%le %le %le]",
         &Is,&k,
         &DbleVal[0],
         &DbleVal[1],
         &DbleVal[2]) == 5)
      {
         SC[Is].B[k].wn[0] = DbleVal[0];
         SC[Is].B[k].wn[1] = DbleVal[1];
         SC[Is].B[k].wn[2] = DbleVal[2];
         SC[Is].RequestStateRefresh = 1;
      }

      if (sscanf(line,"SC[%ld].B[%ld].qn = [%le %le %le %le]",
         &Is,&k,
         &DbleVal[0],
         &DbleVal[1],
         &DbleVal[2],
         &DbleVal[3]) == 6)
      {
         SC[Is].B[k].qn[0] = DbleVal[0];
         SC[Is].B[k].qn[1] = DbleVal[1];
         SC[Is].B[k].qn[2] = DbleVal[2];
         SC[Is].B[k].qn[3] = DbleVal[3];
         SC[Is].RequestStateRefresh = 1;
      }

      if (sscanf(line,"SC[%ld].G[%ld].Pos = [%le %le %le]",
         &Is,&k,
         &DbleVal[0],
         &DbleVal[1],
         &DbleVal[2]) == 5)
      {
         SC[Is].G[k].Pos[0] = DbleVal[0];
         SC[Is].G[k].Pos[1] = DbleVal[1];
         SC[Is].G[k].Pos[2] = DbleVal[2];
         SC[Is].RequestStateRefresh = 1;
      }

      if (sscanf(line,"SC[%ld].G[%ld].PosRate = [%le %le %le]",
         &Is,&k,
         &DbleVal[0],
         &DbleVal[1],
         &DbleVal[2]) == 5)
      {
         SC[Is].G[k].PosRate[0] = DbleVal[0];
         SC[Is].G[k].PosRate[1] = DbleVal[1];
         SC[Is].G[k].PosRate[2] = DbleVal[2];
         SC[Is].RequestStateRefresh = 1;
      }

      if (sscanf(line,"SC[%ld].G[%ld].Ang = [%le %le %le]",
         &Is,&k,
         &DbleVal[0],
         &DbleVal[1],
         &DbleVal[2]) == 5)
      {
         SC[Is].G[k].Ang[0] = DbleVal[0];
         SC[Is].G[k].Ang[1] = DbleVal[1];
         SC[Is].G[k].Ang[2] = DbleVal[2];
         SC[Is].RequestStateRefresh = 1;
      }

      if (sscanf(line,"SC[%ld].G[%ld].AngRate = [%le %le %le]",
         &Is,&k,
         &DbleVal[0],
         &DbleVal[1],
         &DbleVal[2]) == 5)
      {
         SC[Is].G[k].AngRate[0] = DbleVal[0];
         SC[Is].G[k].AngRate[1] = DbleVal[1];
         SC[Is].G[k].AngRate[2] = DbleVal[2];
         SC[Is].RequestStateRefresh = 1;
      }

      if (sscanf(line,"SC[%ld].Whl[%ld].H = %le",
         &Is,&k,
         &DbleVal[0]) == 3)
      {
         SC[Is].Whl[k].H = DbleVal[0];
         SC[Is].RequestStateRefresh = 1;
      }

      if (sscanf(line,"SC[%ld].Whl[%ld].Tcmd = %le",
         &Is,&k,
         &DbleVal[0]) == 3)
      {
         SC[Is].Whl[k].Tcmd = DbleVal[0];
      }

      if (sscanf(line,"SC[%ld].MTB[%ld].Mcmd = %le",
         &Is,&k,
         &DbleVal[0]) == 3)
      {
         SC[Is].MTB[k].Mcmd = DbleVal[0];
      }

      if (sscanf(line,"World[%ld].PosH = [%le %le %le]",
         &Is,
         &DbleVal[0],
         &DbleVal[1],
         &DbleVal[2]) == 4)
      {
         World[Is].PosH[0] = DbleVal[0];
         World[Is].PosH[1] = DbleVal[1];
         World[Is].PosH[2] = DbleVal[2];
      }

      if (sscanf(line,"World[%ld].eph.PosN = [%le %le %le]",
         &Is,
         &DbleVal[0],
         &DbleVal[1],
         &DbleVal[2]) == 4)
      {
         World[Is].eph.PosN[0] = DbleVal[0];
         World[Is].eph.PosN[1] = DbleVal[1];
         World[Is].eph.PosN[2] = DbleVal[2];
      }

      if (sscanf(line,"World[%ld].eph.VelN = [%le %le %le]",
         &Is,
         &DbleVal[0],
         &DbleVal[1],
         &DbleVal[2]) == 4)
      {
         World[Is].eph.VelN[0] = DbleVal[0];
         World[Is].eph.VelN[1] = DbleVal[1];
         World[Is].eph.VelN[2] = DbleVal[2];
      }

      if (sscanf(line,"Orb[%ld].PosN = [%le %le %le]",
         &Is,
         &DbleVal[0],
         &DbleVal[1],
         &DbleVal[2]) == 4)
      {
         Orb[Is].PosN[0] = DbleVal[0];
         Orb[Is].PosN[1] = DbleVal[1];
         Orb[Is].PosN[2] = DbleVal[2];
      }

      if (sscanf(line,"Orb[%ld].VelN = [%le %le %le]",
         &Is,
         &DbleVal[0],
         &DbleVal[1],
         &DbleVal[2]) == 4)
      {
         Orb[Is].VelN[0] = DbleVal[0];
         Orb[Is].VelN[1] = DbleVal[1];
         Orb[Is].VelN[2] = DbleVal[2];
      }

      if (sscanf(line,"CommLink[%ld].Doppler = %le",
         &Is,
         &DbleVal[0]) == 2)
      {
         CommLink[Is].Doppler = DbleVal[0];
      }

      if (sscanf(line,"CommLink[%ld].Delay = %le",
         &Is,
         &DbleVal[0]) == 2)
      {
         CommLink[Is].Delay = DbleVal[0];
      }

      if (sscanf(line,"CommLink[%ld].Carrier = %le",
         &Is,
         &DbleVal[0]) == 2)
      {
         CommLink[Is].Carrier = DbleVal[0];
      }

      if (sscanf(line,"CommLink[%ld].CNR = %le",
         &Is,
         &DbleVal[0]) == 2)
      {
         CommLink[Is].CNR = DbleVal[0];
      }

      if (sscanf(line,"CommLink[%ld].Range = %le",
         &Is,
         &DbleVal[0]) == 2)
      {
         CommLink[Is].Range = DbleVal[0];
      }

      if (sscanf(line,"CommLink[%ld].RangeRate = %le",
         &Is,
         &DbleVal[0]) == 2)
      {
         CommLink[Is].RangeRate = DbleVal[0];
      }

      if (sscanf(line,"CommLink[%ld].PathIsOcculted = %ld",
         &Is,
         &LongVal[0]) == 2)
      {
         CommLink[Is].PathIsOcculted = LongVal[0];
      }
         if (!strncmp(line,"[ENDMSG]",8)) {
            Done = 1;
            //sprintf(line,"[ENDMSG] reached\n");
         }
         if ((size_t)Imsg >= NumBytes && !Done) {
            Done = 1;
            printf("Text frame ended before [ENDMSG]\n");
         }
      }
      if (ShireWriteAll(Socket,Ack,sizeof(Ack)) != 0) {
         printf("Error writing acknowledgement in ReadFromSocket.\n");
         exit(1);
      }
      if (EchoEnabled) printf("MsgLen = %ld\n\n",Imsg);

      if (RequestTimeRefresh) {
         /* Update time variables */
         UTC.Year = Year;
         UTC.doy = doy;
         UTC.Hour = Hour;
         UTC.Minute = Minute;
         UTC.Second = Second;
         DOY2MD(UTC.Year,UTC.doy,&UTC.Month,&UTC.Day);
         CivilTime = DateToTime(UTC.Year,UTC.Month,UTC.Day,UTC.Hour,UTC.Minute,UTC.Second);
         AtomicTime = CivilTime + LeapSec;
         GpsTime = AtomicTime - 19.0;
         DynTime = AtomicTime + 32.184;
         TT.JulDay = TimeToJD(DynTime);
         TimeToDate(DynTime,&TT.Year,&TT.Month,&TT.Day,
            &TT.Hour,&TT.Minute,&TT.Second,DTSIM);
         TT.doy = MD2DOY(TT.Year,TT.Month,TT.Day);
         UTC.JulDay = TimeToJD(CivilTime);
         GpsTimeToGpsDate(GpsTime,&GpsRollover,&GpsWeek,&GpsSecond);
         SimTime = DynTime-DynTime0;
      }

/* .. Refresh SC states that depend on inputs */

      for(Is=0;Is<Nsc;Is++) {
         if (SC[Is].RequestStateRefresh) {
            S = &SC[Is];
            S->RequestStateRefresh = 0;
            if (S->Exists) {
               /* Update  RefOrb */
               O = &Orb[S->RefOrb];
               O->Epoch = DynTime;
               for(i=0;i<3;i++) {
                  S->PosN[i] = O->PosN[i] + S->PosR[i];
                  S->VelN[i] = O->VelN[i] + S->VelR[i];
               }
               RV2Eph(O->Epoch,O->mu,O->PosN,O->VelN,
                  &O->SMA,&O->ecc,&O->inc,&O->RAAN,
                  &O->ArgP,&O->anom,&O->tp,
                  &O->SLR,&O->alpha,&O->rmin,
                  &O->MeanMotion,&O->Period);
               FindCLN(O->PosN,O->VelN,O->CLN,O->wln);

               /* Update Dyn */
               MapJointStatesToStateVector(S);
               D = &S->Dyn;
               MapStateVectorToBodyStates(D->u,D->x,D->h,D->a,D->uf,D->xf,S);
               MotionConstraints(S);
            }
         }
      }
}
