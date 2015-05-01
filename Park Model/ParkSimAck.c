/**
 * ParkSim.c
 * Consists of the simuation for Park native model
 * Created on: Apr 25, 2015
 * Updated on: Aug 30, 2014
 * Author: Saikiran Malyala
 */
#include<stdio.h>
#include<stdlib.h>
#include<time.h>
#include<windows.h>
#include<math.h>

#define t 40000000
#define node_num 10 //0 ----> node_num-1#define pkt 5#define ack 2
int h = 200, macMinBE = 3, macMaxBE = 8, m_b = 2, n_b = 1;
int qStateInd[node_num], csmaStateInd[node_num];
int wakeCount[node_num];
int qSlot[node_num];
int w[node_num], m[node_num], n[node_num];
int inB000[node_num], channel[node_num][pkt + ack + 2];
int in_alpha[node_num], in_beta[node_num], fail_alpha[node_num],
		fail_beta[node_num], succ_beta[node_num];
int out_fail_csma_backoff[node_num], out_fail_csma_retry[node_num],
		out_succ[node_num], in[node_num];
int delayCsma[node_num], delay[node_num];
float eta = 0.5;
int varMax = 1, iterMax = 1;
FILE *fp_r, *fp_al, *fp_be, *fp_booo, *fp_Pwo, *fp_T, *fp_debug, *fp_delay;

void initIter() {
	int i = 0, j = 0;
	srand(0);
	for (i = 0; i < node_num; i++) {
		qStateInd[i] = 1;
		qSlot[i] = 0;
		csmaStateInd[i] = 0;
		wakeCount[i] = 0;
		m[i] = 0;
		n[i] = 0;
		w[i] = 0;
		in[i] = 0;
		inB000[i] = 0;
		in_alpha[i] = 0;
		in_beta[i] = 0;
		fail_alpha[i] = 0;
		fail_beta[i] = 0;
		succ_beta[i] = 0;
		out_fail_csma_backoff[i] = 0;
		out_fail_csma_retry[i] = 0;
		out_succ[i] = 0;
		delayCsma[i] = 0;
		delay[i] = 0;
		for (j = 0; j <= pkt + ack + 1; j++) {
			channel[i][j] = 0;
		}
	}

}
int csma_sense() {

	int i, j = 0;

	for (i = 0; i < node_num; i++) {
		j = channel[i][0] + j;
		if (j > 0) {
			return (0);
		}
	}
	return (1);
}
float randNum_0_1() {
	float randNum = (float) rand() / (float) RAND_MAX;
	if(randNum == 0) {
		return ((float) 2);
	}
	return randNum;
}

int power(int a, int b) {
	int j;
	int c = 1;
	for (j = 0; j < b; j++)
		c = c * a;

	return (c);
}
int MIN(int a, int b) {
	int c;
	c = (((a) < (b)) ? (a) : (b));
	return (c);
}
int randBackOffSlot(int m) {
	return power(2, MIN(macMaxBE, macMinBE + m))
			* ((float) rand() / (float) (RAND_MAX + 1));

}
void leafNode(int node) {
	if (qStateInd[node] == 1) {
		if (qSlot[node] < h) {
			qSlot[node] = qSlot[node] + 1;
			if (qSlot[node] == h) {
				wakeCount[node] = wakeCount[node] + 1;
				if (randNum_0_1() <= eta) {
					csmaStateInd[node] = 1;
					m[node] = 0;
					w[node] = randBackOffSlot(m[node]);
					n[node] = 0;
					qStateInd[node] = 0;
					in[node] = in[node] + 1;
					delay[node] = 0;
				} else {
					qSlot[node] = 0;
				}
			}
		}

	} else if (csmaStateInd[node] == 1) {
		delay[node] = delay[node]+1;
		// CCA Contention slots
		if ((w[node] == 0) && (m[node] == 0) && (n[node] == 0)) {
			inB000[node] = inB000[node] + 1;
		}
		if ((w[node] == 0) && (m[node] != -1) && (m[node] != -2)) {
			in_alpha[node] = in_alpha[node] + 1;
			if (csma_sense() > 0) {
				w[node] = -1;
			} else {
				fail_alpha[node] = fail_alpha[node] + 1;
				if (m[node] < m_b) {
					m[node] = m[node] + 1;
					w[node] = randBackOffSlot(m[node]);
				} else {
					wakeCount[node] = wakeCount[node] + 1;
					if (randNum_0_1() <= eta) {
						csmaStateInd[node] = 1;
						m[node] = 0;
						w[node] = randBackOffSlot(m[node]);
						n[node] = 0;
						qStateInd[node] = 0;
						in[node] = in[node] + 1;
						delay[node] = 0;

					} else {
						qSlot[node] = 0;
						qStateInd[node] = 1;
						m[node] = 0;
						w[node] = 0;
						n[node] = 0;
						csmaStateInd[node] = 0;
					}
					out_fail_csma_backoff[node] = out_fail_csma_backoff[node]
							+ 1;
				}
			}
		}

		else if ((w[node] == -1) && (m[node] != -1) && (m[node] != -2)) {
			in_beta[node] = in_beta[node] + 1;
			if (csma_sense()) {
				succ_beta[node] = succ_beta[node] + 1;
				channel[node][1] = 1;
			} else {
				fail_beta[node] = fail_beta[node] + 1;
				if (m[node] < m_b) {
					m[node] = m[node] + 1;
					w[node] = randBackOffSlot(m[node]);
				} else {
					wakeCount[node] = wakeCount[node] + 1;
					if (randNum_0_1() <= eta) {
						csmaStateInd[node] = 1;
						m[node] = 0;
						w[node] = randBackOffSlot(m[node]);
						n[node] = 0;
						qStateInd[node] = 0;
						in[node] = in[node] + 1;
						delay[node] = 0;
					} else {
						qSlot[node] = 0;
						qStateInd[node] = 1;
						csmaStateInd[node] = 0;
						m[node] = 0;
						w[node] = 0;
						n[node] = 0;

					}
					out_fail_csma_backoff[node] = out_fail_csma_backoff[node]
							+ 1;
				}
			}
		} else if ((w[node] > 0) && (m[node] != -1) && (m[node] != -2)) {
			w[node] = w[node] - 1;
		} else if (m[node] == -2) {
			if (w[node] > 1)
				w[node] = w[node] - 1;
			else {
				if (n[node] < n_b) {
					m[node] = 0;
					w[node] = randBackOffSlot(m[node]);
					n[node] = n[node] + 1;
				} else {
					wakeCount[node] = wakeCount[node] + 1;
					if (randNum_0_1() <= eta) {
						csmaStateInd[node] = 1;
						m[node] = 0;
						n[node] = 0;
						w[node] = randBackOffSlot(m[node]);
						qStateInd[node] = 0;
						in[node] = in[node] + 1;
						delay[node] = 0;
					} else {
						qSlot[node] = 0;
						qStateInd[node] = 1;
						m[node] = 0;
						w[node] = 0;
						n[node] = 0;
						csmaStateInd[node] = 0;
					}
					out_fail_csma_retry[node] = out_fail_csma_retry[node] + 1;
				}

			}
		} else if (m[node] == -1) {
			if (w[node] > 1)
				w[node] = w[node] - 1;
			else {
				wakeCount[node] = wakeCount[node] + 1;
				delayCsma[node] = delayCsma[node] + delay[node];

				if (randNum_0_1() <= eta) {
					delay[node] = 0;
					csmaStateInd[node] = 1;
					m[node] = 0;
					n[node] = 0;
					w[node] = randBackOffSlot(m[node]);
					qStateInd[node] = 0;
					in[node] = in[node] + 1;
				} else {
					qSlot[node] = 0;
					qStateInd[node] = 1;
					m[node] = 0;
					w[node] = 0;
					n[node] = 0;
					csmaStateInd[node] = 0;
				}
				out_succ[node] = out_succ[node] + 1;
			}
		}

	}

}

void sense_collission() {
	int i, j = 0, k, l, n_u;

	for (i = 0; i < node_num; i++) {
		j = channel[i][1] + j;
		if (channel[i][1])
			n_u = i;
	}
	if (j > 1) {
		for (k = 0; k < node_num; k++) {
			if (channel[k][1] && m[k] != -2 && csmaStateInd[k] == 1
					&& m[k] != -1) {
				m[k] = -2;
				w[k] = pkt + ack + 1;
				for (l = 1; l <= pkt; l++)
					channel[k][l] = 1;
			for (l = pkt + 1; l <= pkt + ack + 1; l++)
			 channel[k][l] = 0;

		}
	}
}
else if ((j == 1)) {
	if ((w[n_u] == -1) && (csmaStateInd[n_u] == 1) && m[n_u] != -1
			&& m[n_u] != -2) {
		m[n_u] = -1;
		w[n_u] = pkt + ack + 1;
		for (l = 1; l <= pkt; l++)
		channel[n_u][l] = 1;
		channel[n_u][pkt + 1] = 0;
		for (l = pkt + 2; l <= pkt + ack + 1; l++)
		channel[n_u][l] = 1;

	}

}

}

void shifter() {
int j, k;

for (j = 0; j < node_num; j++) {
	for (k = 0; k <= pkt + ack; k++) {
		channel[j][k] = channel[j][k + 1];
	}
	channel[j][pkt + ack + 1] = 0;
}

}
void logResult() {
int j;
for (j = 0; j < node_num; j++) {

	fprintf(fp_r, "%f\t", (float) out_succ[j] / in[j]);
	//fprintf(fp_r, "%f\t", (float) succ_beta[j]);
	fprintf(fp_al, "%f\t", (float) fail_alpha[j] / in_alpha[j]);
	//fprintf(fp_al, "%f\t", (float) in_beta[j]);
	fprintf(fp_be, "%f\t", (float) fail_beta[j] / in_beta[j]);
	fprintf(fp_booo, "%f\t", (float) inB000[j] / t);
	fprintf(fp_T, "%f\t", (float) in_alpha[j] / t);
	fprintf(fp_Pwo, "%f\t", (float) in[j] / t);
	fprintf(fp_delay, "%f\t", (float) delayCsma[j] / out_succ[j]);
}

}
void debugGen(int time) {
int i = 0;
for (i = 0; i < node_num; i++) {
	fprintf(fp_debug,
			"{t:%d|I:%d|I_N:%d|C:%d|m:%d|w:%d|n:%d|ch:%d|be:%d|fai:%d}\t", time,
			qStateInd[i], qSlot[i], csmaStateInd[i], m[i], w[i], n[i],
			channel[i][0], in_beta[i], fail_beta[i]);
}
fprintf(fp_debug, "\n");
}

int main() {
int i, j, iter, var;

// File pointer initializations
fp_r = fopen("Outputs/rel.txt", "w");
fp_al = fopen("Outputs/alpha.txt", "w");
fp_be = fopen("Outputs/beta.txt", "w");
fp_booo = fopen("Outputs/booo.txt", "w");
fp_Pwo = fopen("Outputs/Pwo.txt", "w");
fp_T = fopen("Outputs/T.txt", "w");
fp_debug = fopen("Outputs/Debug.txt", "w");
fp_delay = fopen("Outputs/Delay.txt", "w");
printf("Process Started for ACK Scenario\n");
for (var = 0; var < varMax; var++) {
	for (iter = 0; iter < iterMax; iter++) {
		initIter();
		//system("cls");
		printf("\nProgress %d-%d\n", var, iter);
		for (i = 0; i < t; i++) {
			//debugGen(i);

			for (j = 0; j < node_num; j++) {
				leafNode(j);
			}
			sense_collission();
			shifter();
		}
	}
	logResult();
	printf("Time: %d", i);
}
printf("\n*************--Process Completed--***************\n");
return (1);
}
