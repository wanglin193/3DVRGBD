function write_xyz_vn(filename, V,N)
    f = fopen( filename, 'w');
    fprintf( f, '%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n', [V,N]');
    fclose(f);
 end
