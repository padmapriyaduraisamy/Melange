#include <iostream>
#include <algorithm>
#include <math.h>
#include <unistd.h>
#include <sys/types.h>
 
#include "controller.hh"
#include "timestamp.hh"

using namespace std;
using namespace alglib;

#define ALPHA          0.875
#define M              0.7
#define DELTA1         1.0
#define DELTA2         2.0 
#define EPSILON        5e3
#define R              6.0
#define MAX_D_PTS      300
#define MAX_RTO        1000.0
#define MIN_RTO        150.0
#define SS_EXIT_THRESH 500
#define MAX_DELAY      100


double ewma (double prev, double cur, double alpha)
{
  if (prev < 0)
    return cur;
  else
    return (alpha * prev + (1-alpha) * cur);
}

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_(debug),
    Dmax_cur (0),
    Dmax_prev (-10),
    dDelay (0),
    epoch_max_delay (0),
    slow_start (true),
    loss_recovery (false),
    new_epoch (true),
    w_delay (MAX_D_PTS, -1),
    Dest (0),
    Dmin (1000),
    rto (1000),
    wnd (1),
    s_wnd (1),
    lock_delay_vars (), 
    loss_rec_seqno (0),
    set_loss_rec_seqno (false),
    RTT_est (-10),
    have_spline (false),
    sender_w (1),
    splineTemp (),
    count (0),
    wind_estimation_tid (-1)
{}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
//  if ( debug_ ) {
    cout << "At time " << timestamp_ms()
	 << " window size is " << sender_w << endl;
// }
  return sender_w; 
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp )
                                    /* in milliseconds */
{
  sender_w --;
  if (set_loss_rec_seqno)
    {
      cout << "First packet after the loss " << sequence_number << endl;
      set_loss_rec_seqno = false;
      loss_rec_seqno = sequence_number;
    }
  if ( debug_ ) {
    cout << "At time " << send_timestamp
	 << " sent datagram " << sequence_number << endl;
  }
}

/* An ack was received */
void Controller::ack_received( const uint64_t sequence_number_acked,
			       /* what sequence number was acknowledged */
			       const uint64_t send_timestamp_acked,
			       /* when the acknowledged datagram was sent (sender's clock) */
			       const uint64_t recv_timestamp_acked,
			       /* when the acknowledged datagram was received (receiver's clock)*/
			       const uint64_t timestamp_ack_received )
                               /* when the ack was received (by sender) */
{
  /* Default: take no action */

  if ( debug_ ) {
    cout << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << endl;
  }

  /* Verus */
  double Dpi = timestamp_ack_received - send_timestamp_acked;
  Dmin = min (Dmin, Dpi);
  RTT_est = ewma (RTT_est, Dpi, (1.0/8.0)); 
  rto = min(MAX_RTO, max ((5 * Dpi), MIN_RTO));

  cout << "Dpi " << Dpi << " Dmin " << Dmin << " RTT_est " << RTT_est << " rto "
<< rto << endl;

  if (loss_recovery && sequence_number_acked == loss_rec_seqno)
    {
      cout << "Leaving Loss Recovery Phase." << endl;
      loss_recovery = false;
    }
  
  if (Dpi > MAX_DELAY) /* Delay Triggered Loss recovery */
    {
      cout << "Entering Loss Recovery Phase." << endl;
      wnd = M * wnd;
      loss_recovery = true;
      set_loss_rec_seqno = true;
      if (slow_start)
        {
          slow_start = false;
          Dest = 0.75 * Dmin * R;
        }
    }

  lock_delay_vars.lock ();
  if (!slow_start && !loss_recovery)
    {
      if (new_epoch)
        {
          cout << "New Epoch." << endl;
          epoch_max_delay = Dpi;
          new_epoch = false;
        }
      else
        epoch_max_delay = max (epoch_max_delay, Dpi);

      Dmax_cur = ewma (Dmax_prev, epoch_max_delay, ALPHA);
      dDelay = Dmax_cur - Dmax_prev;

      cout << "Dmax_cur " << Dmax_cur << " Dmax_prev " << Dmax_prev <<
" epoch_max_delay " << epoch_max_delay << endl;

      int w = floor(wnd);
      w_delay[w] = ewma (w_delay[w], Dpi, ALPHA);
      cout << "w is " << w << " w_delay[w] is " << w_delay[w] << endl;
    }
  if (slow_start && !loss_recovery)
    {
      cout << "In Slow Start." << endl;
      if (Dpi > SS_EXIT_THRESH)
        {
          cout << "Leaving slow start and entering loss recovery." << endl;
          loss_recovery = true;
          slow_start = false;
          Dest = 0.75 * Dmin * R;
        }
      else 
	{
          wnd++;
          sender_w = wnd;
        }
    }
  if (loss_recovery) 
    {
      cout << "In Loss recovery." << endl;
      wnd += 1/wnd;
      sender_w = wnd;
    }
  
  lock_delay_vars.unlock ();
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms( void )
{
  return rto;
}

void* Controller::wind_estimation_thread (void* arg)
{
  Controller &ctrl = *(Controller*)arg;
  while (true)
    {
//strt:
      ctrl.lock_delay_vars.lock ();    
      struct timeval startTime, endTime;
      gettimeofday(&startTime,NULL);
      
      double Dest_next;
      if (ctrl.slow_start || ctrl.loss_recovery)
        {
          ctrl.lock_delay_vars.unlock ();
          usleep (1);
          continue;
        }
  
      if (ctrl.Dmax_cur/ctrl.Dmin > R)
        Dest_next = ctrl.Dest - DELTA2;
      else if (ctrl.dDelay > 0)
        Dest_next = max (ctrl.Dmin, (ctrl.Dest-DELTA1));
      else
        Dest_next = ctrl.Dest + DELTA2;

      //ctrl.Dmax_prev = ctrl.Dmax_cur;
      ctrl.Dmax_prev = ewma (ctrl.Dmax_prev, ctrl.Dmax_cur, 0.2);
  
      int i;
      vector<int>x;
      vector<double> y;
      double* xs = NULL;
      double* ys = NULL;
      //int max_i = 1;
      int cnt = 0;
      //int N = 15;
      int maxW = MAX_D_PTS;
  
      for (i=0; i<maxW; i++)
        {
          if (ctrl.w_delay[i]>0)
            {
              x.push_back(i);
              y.push_back(ctrl.w_delay[i]);
              cnt++;
      ctrl.count++;
      if (ctrl.count == 5)
        ctrl.have_spline = true;
            }
          //max_i = i;
        }
      xs = (double*)malloc(cnt*sizeof(double));
      ys = (double*)malloc(cnt*sizeof(double));
      for (int k=0; k<cnt; k++)
        {
          xs[i] = (double)x[i];
          ys[i] = (double)y[i];
        }
     
      real_1d_array xi;
      real_1d_array yi;
  
      xi.setcontent(cnt, xs);
      yi.setcontent(cnt, ys);
      spline1dfitreport rep;
      ae_int_t info;
      /*while (max_i/N < 5)
        {
          cout << "Fraction " << max_i/N << endl;
          N--;
          if (N<1)
            {
              cout << "Bad thread." << endl;
              ctrl.slow_start = true;
              ctrl.wnd = 1;
              ctrl.lock_delay_vars.unlock ();
              goto strt;
             } 
         }*/
      if (ctrl.have_spline)
        {
          try
            {
              spline1dfitpenalized(xi, yi,/*max_i/N*/ max (cnt, 5), 2.0, info, ctrl.splineTemp, rep);
              ctrl.have_spline = true;
            }
          catch (alglib::ap_error exc)
            {
              cout << exc.msg.c_str() << endl;
              ctrl.have_spline = false;
            }
        } 
      double w_next = ctrl.calcDelayCurve (Dest_next);
     
// Where is inverse calculation?
// Dest calculation from curve

      int n = ceil (ctrl.RTT_est / (EPSILON / 1000)); 
      if (n>1)
        ctrl.s_wnd = max (0.0, w_next + ((2-n)/(n-1))* ctrl.wnd);
      else
        ctrl.s_wnd = max (0.0, w_next - ctrl.wnd);

      ctrl.sender_w += ctrl.s_wnd;
      ctrl.wnd = w_next;
      ctrl.new_epoch = true;

      gettimeofday(&endTime,NULL);
      useconds_t difftime = (endTime.tv_sec - startTime.tv_sec) * 1000000 +
                            (endTime.tv_usec - startTime.tv_usec);
      cout << "sleeping." << endl;
      usleep (EPSILON - difftime);
      ctrl.lock_delay_vars.unlock ();
    }
    return NULL;
}

void Controller :: timeout_event (void) 
{
  cout << "Time-out event." << endl;
  wnd = 1;
  slow_start = true;
}

double Controller :: calcDelayCurve (double Dest_next)
{
  for (int w = 2; w < MAX_D_PTS; w++)
    {
      if (!have_spline)
        {
          if (w_delay[w] > Dest_next)
            return(w-1);
        }
      else
        {
          if (spline1dcalc(splineTemp, w) > Dest_next)
            return (w-1);
        }
     }
  if (!have_spline)
    return calcDelayCurve (Dest_next - DELTA2);
  else
    return -1000.0;
}
