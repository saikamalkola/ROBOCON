
boolean det_junction()
{
  for (int i = 0; i < 6; i++)
  {
    if (Xp == imp_node_x[i] && Yp == imp_node_y[i])
    {
      return true;
    }
  }
  return false;
}

void det_new_dir()
{
  if (Xp < Xf && Yp == Yf)
  {
    dir = -2;
    //Right follow
  }
  else if (Xp > Xf && Yp == Yf)
  {
    dir = 2;
    //left_follow
  }
  else if (Yp < Yf && Xp == Xf)
  {
    if (Xp == 0)
    {
      dir = -1;
      //Back Follow
    }
    else
    {
      dir = 2;
      //left follow
    }
  }
  else if (Yp > Yf && Xp == Xf)
  {
    if (Xp == 0)
    {
      dir = 1;
      //Front Follow
    }
    else
    {
      dir = 2;
      //left follow
    }
  }
  else if (Xp != Xf && Yp != Yf)
  {
    if (Xp < Xf && Yp < Yf)
    {
      dir = -1;
    }
    else if (Xp < Xf && Yp > Yf)
    {
      dir = 1;
    }
    else if (Xp > Xf && Yp < Yf)
    {
      dir = 2;
    }
    else if (Xp > Xf && Yp > Yf)
    {
      dir = 2;
    }
  }
}

void update_position()
{
  if (dir == 1 && jun_data[0] == 1)
  {
    Xp = Xp;
    Yp = Yp - 1;
  }
  else if (dir == -1 && jun_data[1] == 1)
  {
    Xp = Xp;
    Yp = Yp + 1;
  }
  else if (dir == 2  && jun_data[2] == 1)
  {
    Xp = Xp - 1;
    Yp = Yp;
  }
  else if (dir == -2  && jun_data[3] == 1)
  {
    Xp = Xp + 1;
    Yp = Yp;
  }
}
