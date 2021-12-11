//
// Created by bddwy on 2021/12/10.
//

#include "Hungary.h"

using namespace std;

int p[robot_num];         //记录当前右侧元素所对应的左侧元素
int nowp[robot_num];      //记录右侧元素是否已被访问过

int HZ[N], LZ[N];

struct Pair {
    int x, y;
    Pair(int x = 0, int y = 0) :x(x), y(y) {}
    bool operator < (const Pair& b) const {
        return HZ[this->x] == HZ[b.x] ? LZ[this->y] < LZ[b.y] : HZ[this->x] < HZ[b.x];
    }
}Pt[N * N];

int FH[N], FL[N];
int Maxx, Mx;
double st[N * N];
int used[N * N], tot;

void storeP()
{
    for (int i = 0; i < robot_num; i++)
        nowp[i] = p[i];
}

void dfs(int s, double t, double sum) {
    if (sum > Mx) {
        Mx = sum;
        used[0] = tot;//记录最好方案
        for (int i = 1; i <= tot; i++) used[i] = st[i];
    }
    if (Mx == Maxx) return;//已经找到满意解
    if (t - s + 1 + sum <= Mx) return;//乐观估计不如目前最优解
    if (s > t) return;
    if (!FH[Pt[s].x] && !FL[Pt[s].y]) { //选s点
        FH[Pt[s].x] = 1;
        FL[Pt[s].y] = 1;
        st[++tot] = s;
        dfs(s + 1, t, sum + 1);
        --tot;
        FH[Pt[s].x] = 0;
        FL[Pt[s].y] = 0;
    }
    dfs(s + 1, t, sum);//不选s点
}

double calc(double b[N][N], int n) {
    double a[N][N];
    int v[N][N];
    for (int i = 1; i <= n; i++)
        for (int j = 1; j <= n; j++)
            a[i][j] = b[i][j];
    for (int i = 1; i <= n; i++) {
        double t = a[i][1];
        for (int j = 2; j <= n; j++)
            if (a[i][j] < t) t = a[i][j];
        for (int j = 1; j <= n; j++)
            a[i][j] -= t;
    }
    for (int i = 1; i <= n; i++) {
        double t = a[1][i];
        for (int j = 2; j <= n; j++)
            if (a[j][i] < t) t = a[j][i];
        for (int j = 1; j <= n; j++)
            a[j][i] -= t;
    }
    //先让每行每列都有0
//Out(a,v,n);
    int H[N], L[N];
    while (1) {

        for (int i = 1; i <= n; i++) {
            H[i] = L[i] = 0; //H[i]，第i行有多少个0，L为列
            FH[i] = FL[i] = 0; //FH[i],第i行有没有画O
            for (int j = 1; j <= n; j++) v[i][j] = 0;//v[i][j] = 1代表‘O’,-1代表‘X’
        }
        for (int i = 1; i <= n; i++)
            for (int j = 1; j <= n; j++)
                if (a[i][j] == 0) {
                    H[i]++; L[j]++;
                }
        int cnt = 0;
        while (1) {
            int tpcnt = cnt;
            for (int i = 1; i <= n; i++) //找每行单独的0画‘O’，同列画‘X’
                if (H[i] == 1) {
                    int t = 1;
                    while (a[i][t] || v[i][t]) t++;
                    v[i][t] = 1;
                    cnt++;  //cnt记有几个‘O’
                    H[i]--; L[t]--;
                    FH[i] = 1; FL[t] = 1;
                    for (int j = 1; j <= n; j++)
                        if (a[j][t] == 0 && j != i && v[j][t] == 0) {
                            v[j][t] = -1;
                            H[j]--; L[t]--;
                        }
                }
            for (int i = 1; i <= n; i++) //对称的
                if (L[i] == 1) {
                    int t = 1;
                    while (a[t][i] || v[t][i]) t++;
                    v[t][i] = 1;
                    cnt++;
                    H[t]--; L[i]--;
                    FH[t] = 1; FL[i] = 1;
                    for (int j = 1; j <= n; j++)
                        if (a[t][j] == 0 && j != i && v[t][j] == 0) {
                            v[t][j] = -1;
                            H[t]--; L[j]--;
                        }
                }
            if (tpcnt == cnt) break;
        }
        //Out(a,v,n);
        int top = 0;
        for (int i = 1; i <= n; i++)
            for (int j = 1; j <= n; j++)
                if (a[i][j] == 0 && v[i][j] == 0) {
                    Pt[++top] = Pair(i, j);
                    HZ[i]++;
                    LZ[j]++;
                }
        sort(Pt + 1, Pt + top + 1);//同行同列少的排前面
        Maxx = n - cnt;
        Mx = 0; used[0] = 0;
        dfs(1, top, 0);//对剩下的0进行试探画‘O’
        cnt += Mx;
        for (int i = 1; i <= used[0]; i++) {
            v[Pt[used[i]].x][Pt[used[i]].y] = 1;
            FH[Pt[used[i]].x] = 1;
            FL[Pt[used[i]].y] = 1;
        }
        //Out(a,v,n);
        if (cnt == n) { //已经找到
            double ans = 0;
            for (int i = 1; i <= n; i++)
                for (int j = 1; j <= n; j++)
                    if (v[i][j] == 1) {
                        ans += b[i][j];
                        p[i - 1] = j - 1;
                    }
            return ans;
        }
        int flagx[N], flagy[N]; //对号标记
        for (int i = 1; i <= n; i++) flagx[i] = flagy[i] = 0;
        int cas = 1;//时间戳，每次只检查新增对号行/列
        for (int i = 1; i <= n; i++)
            if (!FH[i]) flagx[i] = cas;
        bool chang = 1;
        while (chang) {
            chang = 0;
            cas++;
            for (int i = 1; i <= n; i++)
                if (flagx[i] == cas - 1)
                    for (int j = 1; j <= n; j++)
                        if (v[i][j] == -1) {
                            flagy[j] = cas;
                            chang = 1;
                        }
            for (int i = 1; i <= n; i++)
                if (flagy[i] == cas - 1)
                    for (int j = 1; j <= n; j++)
                        if (v[j][i] == 1) {
                            flagx[j] = cas;
                            chang = 1;
                        }
        }
        double Mi = ~0u >> 2;
        for (int i = 1; i <= n; i++)
            for (int j = 1; j <= n; j++)
                if (flagx[i] && !flagy[j] && Mi > a[i][j])
                    Mi = a[i][j];  //未划线找最小的
        for (int i = 1; i <= n; i++)
            for (int j = 1; j <= n; j++)
                if (flagx[i] && !flagy[j]) //未划线
                    a[i][j] -= Mi;
                else if (!flagx[i] && flagy[j]) //线交点
                    a[i][j] += Mi;
    }
}
