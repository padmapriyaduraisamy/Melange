#include <iostream>
#include <algorithm>
#include <math.h>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

#define ALPHA          0.875
#define M              0.7
#define DELTA1         1
#define DELTA2         2 
#define EPSILON        5
#define N              15
#define R              2
#define MAX_D_PTS      300
#define MAX_RTO        1000.0
#define MIN_RTO        150.0
#define SS_EXIT_THRESH 500


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
    sent_list (), 
    wind_estimation_tid (-1)
{}

/* Get current window size, in datagrams */
unsigned int Controller::window_size( void )
{
  if ( debug_ ) {
    cerr << "At time " << timestamp_ms()
	 << " window size is " << s_wnd << endl;
  }
  return s_wnd; 
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp )
                                    /* in milliseconds */
{
  sent_list.push_back (sequence_number);
  if ( debug_ ) {
    cerr << "At time " << send_timestamp
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
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << send_timestamp_acked
	 << ", received @ time " << recv_timestamp_acked << " by receiver's clock)"
	 << endl;
  }

  /* Verus */
  if (sequence_number_acked == (sent_list[0] + 1))
    {
      sent_list.erase (sent_list.begin());
      if (loss_recovery)
        loss_recovery = false;
    }
  else if (sequence_number_acked > (sent_list [0] + 1)) /* packet loss */
    {
      wnd = M * wnd;
      loss_recovery = true;
      if (slow_start)
        {
          slow_start = false;
          Dest = 0.75 * Dmin * R;
        }
      sent_list.clear ();
    }
  double Dpi = recv_timestamp_acked - send_timestamp_acked;
  Dmin = min (Dmin, Dpi);
  rto = min(MAX_RTO, max ((5 * Dpi), MIN_RTO));

  lock_delay_vars.lock ();
  if (!slow_start && !loss_recovery)
    {
      if (new_epoch)
        {
          epoch_max_delay = Dpi;
          new_epoch = false;
        }
      else
        epoch_max_delay = max (epoch_max_delay, Dpi);

      Dmax_cur = ewma (Dmax_prev, epoch_max_delay, ALPHA);
      dDelay = Dmax_cur - Dmax_prev;

      int w = floor(wnd);
      w_delay[w] = ewma (w_delay[w], Dpi, ALPHA);
    }
  if (slow_start && !loss_recovery)
    {
      if (Dpi > SS_EXIT_THRESH)
        {
          loss_recovery = true;
          slow_start = false;
          Dest = 0.75 * Dmin * R;
        }
      else 
	wnd++;
    }
  if (loss_recovery) 
    {
      wnd += 1/wnd;
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
  ctrl.dDelay++;
  }
  return NULL;
}
