
using UnityEngine;

public class Fluid
{
    int N;
    int iter = 4;

    float deltaTime;
    float diffusion;
    float viscocity;

    float[] s;
    float[] density;

    float[] vx;
    float[] vy;

    float[] vx0;
    float[] vy0;


    public float[] Density => density;

    public Fluid(int size, int diffusion, int viscocity, float deltaTime)
    {
        this.N = size;
        this.deltaTime = deltaTime;
        this.diffusion = diffusion;
        this.viscocity = viscocity;
        this.s = new float[size * size];
        this.density = new float[size * size];
        vx = new float[size * size];
        vy = new float[size * size];
        vx0 = new float[size * size];
        vy0 = new float[size * size];
    }

    public void Step(float deltaTime)
    {
        this.deltaTime = deltaTime;

        Diffuse(1, vx0, vx, viscocity, deltaTime);
        Diffuse(2, vy0, vy, viscocity, deltaTime);

        Project(vx0, vy0, vx, vy);

        Advect(1, vx, vx0, vx0, vy0, deltaTime);
        Advect(2, vy, vy0, vx0, vy0, deltaTime);

        Project(vx, vy, vx0, vy0);
        Diffuse(0, s, density, diffusion, deltaTime);
        Advect(0, density, s, vx, vy, deltaTime);

    }

    public void AddDencity(int x, int y, float amount)
    {
        density[Index(x, y)] += amount;
    }
    public void AddVelocity(int x, int y, float amountX, float amountY)
    {
        int index = Index(x, y);
        vx[index] += amountX;
        vy[index] += amountY;
    }
    void Diffuse(int b, float[] x, float[] x0, float diff, float dt)
    {
        float a = dt * diff * (N - 2) * (N - 2);
        LinSolve(b, x, x0, a, 1 + 6 * a);
    }
    void LinSolve(int b, float[] x, float[] x0, float a, float c)
    {
        float cRecip = 1.0f / c;
        for (int t = 0; t < iter; t++)
        {
            for (int j = 1; j < N - 1; j++)
            {
                for (int i = 1; i < N - 1; i++)
                {
                    x[Index(i, j)] =
                      (x0[Index(i, j)] +
                        a *
                          (x[Index(i + 1, j)] +
                            x[Index(i - 1, j)] +
                            x[Index(i, j + 1)] +
                            x[Index(i, j - 1)])) *
                      cRecip;
                }
            }
            SetBounds(b, x);
        }
    }

    void Project(float[] velocX, float[] velocY, float[] p, float[] div)
    {
        for (int j = 1; j < N - 1; j++)
        {
            for (int i = 1; i < N - 1; i++)
            {
                div[Index(i, j)] =
                  (-0.5f *
                    (velocX[Index(i + 1, j)] -
                      velocX[Index(i - 1, j)] +
                      velocY[Index(i, j + 1)] -
                      velocY[Index(i, j - 1)])) /
                  N;
                p[Index(i, j)] = 0;
            }
        }

        SetBounds(0, div);
        SetBounds(0, p);
        LinSolve(0, p, div, 1, 6);

        for (int j = 1; j < N - 1; j++)
        {
            for (int i = 1; i < N - 1; i++)
            {
                velocX[Index(i, j)] -= 0.5f * (p[Index(i + 1, j)] - p[Index(i - 1, j)]) * N;
                velocY[Index(i, j)] -= 0.5f * (p[Index(i, j + 1)] - p[Index(i, j - 1)]) * N;
            }
        }

        SetBounds(1, velocX);
        SetBounds(2, velocY);
    }

    void Advect(int b, float[] d, float[] d0, float[] velocX, float[] velocY, float dt)
    {
        float i0, i1, j0, j1;

        float dtx = dt * (N - 2);
        float dty = dt * (N - 2);

        float s0, s1, t0, t1;
        float tmp1, tmp2, tmp3, x, y;

        float Nfloat = N;
        float ifloat, jfloat;
        int i, j, k;

        for (j = 1, jfloat = 1; j < N - 1; j++, jfloat++)
        {
            for (i = 1, ifloat = 1; i < N - 1; i++, ifloat++)
            {
                tmp1 = dtx * velocX[Index(i, j)];
                tmp2 = dty * velocY[Index(i, j)];
                x = ifloat - tmp1;
                y = jfloat - tmp2;

                if (x < 0.5) x = 0.5f;
                if (x > Nfloat + 0.5) x = Nfloat + 0.5f;
                i0 = Mathf.Floor(x);
                i1 = i0 + 1.0f;
                if (y < 0.5) y = 0.5f;
                if (y > Nfloat + 0.5) y = Nfloat + 0.5f;
                j0 = Mathf.Floor(y);
                j1 = j0 + 1.0f;

                s1 = x - i0;
                s0 = 1.0f - s1;
                t1 = y - j0;
                t0 = 1.0f - t1;

                int i0i = (int)(i0);
                int i1i = (int)(i1);
                int j0i = (int)(j0);
                int j1i = (int)(j1);

                d[Index(i, j)] =
                  s0 * (t0 * d0[Index(i0i, j0i)] + t1 * d0[Index(i0i, j1i)]) +
                  s1 * (t0 * d0[Index(i1i, j0i)] + t1 * d0[Index(i1i, j1i)]);
            }
        }

        SetBounds(b, d);
    }


    void SetBounds(int b, float[] x)
    {
        for (int i = 1; i < N - 1; i++)
        {
            x[Index(i, 0)] = b == 2 ? -x[Index(i, 1)] : x[Index(i, 1)];
            x[Index(i, N - 1)] = b == 2 ? -x[Index(i, N - 2)] : x[Index(i, N - 2)];
        }
        for (int j = 1; j < N - 1; j++)
        {
            x[Index(0, j)] = b == 1 ? -x[Index(1, j)] : x[Index(1, j)];
            x[Index(N - 1, j)] = b == 1 ? -x[Index(N - 2, j)] : x[Index(N - 2, j)];
        }

        x[Index(0, 0)] = 0.5f * (x[Index(1, 0)] + x[Index(0, 1)]);
        x[Index(0, N - 1)] = 0.5f * (x[Index(1, N - 1)] + x[Index(0, N - 2)]);
        x[Index(N - 1, 0)] = 0.5f * (x[Index(N - 2, 0)] + x[Index(N - 1, 1)]);
        x[Index(N - 1, N - 1)] = 0.5f * (x[Index(N - 2, N - 1)] + x[Index(N - 1, N - 2)]);
    }
    int Index(int x, int y) => Mathf.Clamp(x + y * N, 0, N * N - 1);
}
