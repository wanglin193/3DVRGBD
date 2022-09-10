function write_obj_vf(filename, V,F)
f = fopen( filename, 'w');
fprintf( f, 'v %7.6f %7.6f %7.6f\n', V');
fprintf( f,'f %d %d %d\n', F');    
fclose(f);
end