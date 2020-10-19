
using UnityEngine;
public enum PARTICLES { AIR = 0, SAND = 1, water = 2 }
public class SandSimulation
{
    PARTICLES[,] pixelArray;
    int height;
    int width;
    public SandSimulation(int w, int h)
    {
        width = w;
        height = h;
        pixelArray = new PARTICLES[w, h]; // [y,x] positioning due to c# multi dimentional array construction
    }
    bool switcher = false;
    public void ItterateSimulation()
    {
        switcher = !switcher;
        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                if (pixelArray[y, x] == PARTICLES.SAND && y > 0)
                {
                    if (pixelArray[y - 1, x] != PARTICLES.SAND)
                    {
                        pixelArray[y, x] = pixelArray[y - 1, x];
                        pixelArray[y - 1, x] = PARTICLES.SAND;
                    }
                    else if (isIndexWithinBounds(x - 1, y - 1, width, height) && pixelArray[y - 1, x - 1] != PARTICLES.SAND)
                    {
                        pixelArray[y, x] = pixelArray[y - 1, x - 1];
                        pixelArray[y - 1, x - 1] = PARTICLES.SAND;
                    }
                    else if (isIndexWithinBounds(x + 1, y - 1, width, height) && pixelArray[y - 1, x + 1] != PARTICLES.SAND)
                    {
                        pixelArray[y, x] = pixelArray[y - 1, x + 1];
                        pixelArray[y - 1, x + 1] = PARTICLES.SAND;
                    }
                }
                else if (pixelArray[y, x] == PARTICLES.water)
                {
                    if (isIndexWithinBounds(x, y - 1, width, height) && pixelArray[y - 1, x] == 0)
                    {
                        pixelArray[y, x] = 0;
                        pixelArray[y - 1, x] = PARTICLES.water;
                    }
                    else if (isIndexWithinBounds(x - 1, y - 1, width, height) && pixelArray[y - 1, x - 1] == 0)
                    {
                        pixelArray[y, x] = 0;
                        pixelArray[y - 1, x - 1] = PARTICLES.water;
                    }
                    else if (isIndexWithinBounds(x + 1, y - 1, width, height) && pixelArray[y - 1, x + 1] == 0)
                    {
                        pixelArray[y, x] = 0;
                        pixelArray[y - 1, x + 1] = PARTICLES.water;
                    }
                    else if (isIndexWithinBounds(x - 1, y, width, height) && pixelArray[y, x - 1] == 0)
                    {
                        pixelArray[y, x] = 0;
                        pixelArray[y, x - 1] = PARTICLES.water;
                    }
                    else if (isIndexWithinBounds(x + 1, y, width, height) && pixelArray[y, x + 1] == 0)
                    {
                        pixelArray[y, x] = 0;
                        pixelArray[y, x + 1] = PARTICLES.water;
                    }
                }
            }
        }
    }
    public void AddPixelToArray(Vector3 mouseScreenPosition, int type = 1)
    {
        Vector2Int centreIndex = GetScreenPointToArrayIndex(mouseScreenPosition);
        for (int x = -3; x < 3; x++)
        {
            for (int y = -3; y < 3; y++)
            {
                Vector2Int index = centreIndex + new Vector2Int(x, y);
                if (isIndexWithinBounds(index, width, height))
                {
                    pixelArray[index.y, index.x] = (PARTICLES)type;
                }
            }
        }
    }

    bool isIndexWithinBounds(Vector2Int index, int width, int height)
    {
        if (index.x < 0 || index.y < 0 || index.x >= width || index.y >= height)
        {
            return false;
        }
        return true;
    }
    bool isIndexWithinBounds(int x, int y, int width, int height)
    {
        if (x < 0 || y < 0 || x >= width || y >= height)
        {
            return false;
        }
        return true;
    }

    Vector2Int GetScreenPointToArrayIndex(Vector2 mousePos)
    {
        Vector2 screenRect = new Vector2(Screen.width, Screen.height);
        float scale = screenRect.x / screenRect.y;

        Vector2 uv;
        uv.x = (mousePos.x / screenRect.x) * 2 - 1;
        uv.y = (mousePos.y / screenRect.y) * 2 - 1;
        uv.x *= scale;
        Vector2Int UVi = new Vector2Int();
        UVi.x = (int)(uv.x * height / 2);
        UVi.y = (int)(uv.y * height / 2);
        UVi.x += width / 2;
        UVi.y += height / 2;

        return UVi;
    }

    public PARTICLES[,] PixelArray => pixelArray;
}
