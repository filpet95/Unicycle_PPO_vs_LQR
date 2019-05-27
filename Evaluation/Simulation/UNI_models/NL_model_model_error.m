function f = NL_model_model_error(dalpha_w,dalpha_d,dphi,dtheta,phi,theta,u_w,u_d)
%NL_MODEL_MODEL_ERROR
%    F = NL_MODEL_MODEL_ERROR(DALPHA_W,DALPHA_D,DPHI,DTHETA,PHI,THETA,U_W,U_D)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    17-Apr-2019 08:11:58

t2 = cos(theta);
t3 = t2.^2;
t4 = sin(theta);
t5 = theta.*2.0;
t6 = sin(t5);
t7 = t4.^2;
t8 = sin(phi);
t9 = dphi.^2;
t10 = t3.^2;
t11 = dtheta.^2;
t12 = cos(phi);
t13 = t2.*5.31879230650742e140;
t14 = t2.*t4.*3.254903495942177e139;
t15 = t3.*7.306434091120242e140;
t18 = t2.*t3.*2.508840316761623e140;
t19 = t10.*4.078536287403793e140;
t20 = t2.*t3.*t4.*1.533212635875915e139;
t16 = t13+t14+t15-t18-t19-t20+2.854701804456914e140;
t17 = 1.0./t16;
f = [t17.*(dalpha_d.*2.975192992638987e139+dalpha_w.*4.25484044962442e154+t8.*4.049221298246938e141+t9.*1.888160765563855e138-t12.*8.213157514992533e140-u_d.*1.13460306224333e140-u_w.*3.97296216212454e154+dalpha_d.*dphi.*3.867231693914234e137+dphi.*dtheta.*6.415288971073338e138-dalpha_d.*t2.*1.370258005105169e147-dalpha_d.*t3.*2.098565885650946e150+dalpha_d.*t4.*1.141040397046672e152+dalpha_d.*t6.*5.705195049557236e151+dalpha_w.*t2.*9.140680685603927e154+dalpha_w.*t3.*1.51563330088521e155+dalpha_w.*t4.*1.375535024158091e153+dalpha_w.*t6.*2.425664045284325e153+t2.*t8.*7.151672605554629e141+t2.*t9.*1.046680027172695e152-t3.*t8.*2.858311154019027e152+t3.*t9.*3.883921635398497e152+t4.*t8.*2.742792635382849e154+t2.*t11.*6.844565917168953e152+t4.*t9.*9.109433296205108e153-t3.*t12.*2.642026717397377e154+t4.*t11.*2.42020303201989e154+t6.*t9.*8.564125777160662e152+t6.*t11.*5.596140485331469e153-t9.*t10.*5.825818811406577e152+t10.*t12.*2.646008325919495e154+t2.*u_d.*5.225539763310439e147+t3.*u_d.*8.002974214008802e150-t4.*u_d.*4.351408234140024e152-t6.*u_d.*2.175701472119118e152-t2.*u_w.*8.535121100292403e154-t3.*u_w.*1.415224337402449e155-t4.*u_w.*1.284407410421126e153-t6.*u_w.*2.264966591353912e153+dalpha_d.*dphi.*t3.*3.978238537879886e151+dalpha_d.*dphi.*t4.*6.508584520945261e152+dalpha_d.*dphi.*t6.*1.754059253858779e152-dalpha_d.*dphi.*t10.*3.975483279507795e151+dalpha_d.*dtheta.*t2.*4.123041317724613e151-dalpha_d.*dtheta.*t4.*7.58068833119937e149+dphi.*dtheta.*t2.*4.026117629919779e152+dphi.*dtheta.*t3.*1.349801999700027e153-dphi.*dtheta.*t4.*7.410933821563761e150+dphi.*dtheta.*t6.*1.230172765781913e151-dphi.*dtheta.*t10.*1.349349149863922e153-dalpha_d.*t2.*t3.*2.09533478671777e150-dalpha_d.*t4.*t7.*1.141040397046672e152+dalpha_w.*t2.*t3.*3.659094230865452e154-dalpha_w.*t4.*t7.*1.375535024158078e153-t2.*t3.*t8.*5.048305358662195e152+t2.*t3.*t9.*7.775437410831911e152+t2.*t4.*t8.*1.552947816324958e154-t2.*t3.*t11.*6.839871623672643e152-t2.*t4.*t12.*2.355356734588204e155-t3.*t4.*t12.*4.329740188164621e155-t4.*t7.*t8.*2.742792635382849e154-t4.*t7.*t9.*9.109433296205108e153-t2.*t9.*t10.*1.30256735907088e153-t4.*t7.*t11.*1.819491012416316e154+t4.*t9.*t10.*1.783081068169237e154+t2.*t3.*u_d.*7.990652274715914e150+t4.*t7.*u_d.*4.351408234140024e152-t2.*t3.*u_w.*3.416683445359244e154+t4.*t7.*u_w.*1.284407410421114e153+dalpha_d.*dphi.*t2.*t3.*1.272833823944128e148-dalpha_d.*dphi.*t4.*t7.*6.508584520945261e152-dalpha_d.*dtheta.*t2.*t3.*4.123042801798376e151+dalpha_d.*dtheta.*t4.*t7.*7.580688331306876e149-dphi.*dtheta.*t2.*t3.*4.026117630032683e152+dphi.*dtheta.*t4.*t7.*7.41093382166874e150+t2.*t3.*t4.*t9.*1.614893818973801e154-t2.*t3.*t4.*t12.*7.038711214292797e155+dalpha_d.*dphi.*t2.*t3.*t4.*1.058079204412001e153-dphi.*dtheta.*t2.*t3.*t4.*7.406087519401072e151).*(-7.894919286223335e-14);(t17.*(dalpha_d.*-3.976350928812287e148+dalpha_w.*4.095537102200387e142+t9.*3.53341632749019e140-t11.*6.09463593748873e140+u_d.*1.516399087906823e149-u_w.*3.824212478297658e142+dalpha_d.*dtheta.*5.35255800123565e142+dphi.*dtheta.*5.226731064105818e143-dalpha_d.*t2.*8.167311075834716e148-dalpha_d.*t3.*1.093592292908989e149+dalpha_d.*t4.*7.814404433340461e142-dalpha_d.*t6.*2.26675038035727e147+dalpha_d.*t10.*6.038918148043034e148+dalpha_w.*t2.*2.028519012523504e142+dalpha_w.*t3.*6.254771048713499e145-dalpha_w.*t4.*5.665820275323891e147-dalpha_w.*t6.*1.705216522120763e147-t2.*t8.*1.032582083539439e150-t2.*t9.*4.814962861694828e146-t3.*t8.*1.823730652368307e150-t3.*t9.*1.663580748103668e147+t4.*t8.*1.063536145147038e145+t4.*t9.*8.824367529810231e144+t2.*t12.*2.094418137599587e149+t3.*t11.*8.959239605400564e146-t4.*t11.*1.120090641148718e142-t6.*t9.*1.470509995842168e145-t4.*t12.*3.596165711769039e144+t6.*t11.*1.559605188667322e145+t8.*t10.*8.602420862842839e149+t9.*t10.*1.663039727847443e147+t10.*t11.*6.844694562052752e140+t2.*u_d.*3.114640354377966e149+t3.*u_d.*4.170462781574598e149-t4.*u_d.*2.980057838806486e143+t6.*u_d.*8.644353254587892e147-t10.*u_d.*2.302968257978096e149-t2.*u_w.*1.894131960369399e142-t3.*u_w.*5.840399658578102e145+t4.*u_w.*5.290466196739964e147+t6.*u_w.*1.592247888217151e147-dalpha_d.*dphi.*t2.*9.861745559947763e145-dalpha_d.*dphi.*t3.*2.700513071525409e136+dalpha_d.*dphi.*t4.*1.810204597608275e144-dalpha_d.*dphi.*t6.*6.904482441186352e140-dalpha_d.*dtheta.*t3.*1.324275726455247e143-dalpha_d.*dtheta.*t6.*1.370741034565082e147+dalpha_d.*dtheta.*t10.*5.057787195619456e142+dphi.*dtheta.*t2.*4.333216010699254e144-dphi.*dtheta.*t3.*2.460902808975648e143-dphi.*dtheta.*t4.*3.863814535974288e148+dphi.*dtheta.*t6.*7.861718977374916e138+dalpha_d.*t2.*t3.*3.852468403895993e148+dalpha_d.*t4.*t7.*6.998635290438151e142+dalpha_w.*t2.*t3.*1.040435583955176e146+dalpha_w.*t4.*t7.*5.665820275323891e147+t2.*t3.*t8.*4.870623656241272e149+t2.*t3.*t9.*4.814966981728888e146+t2.*t4.*t8.*3.560710537422089e145-t2.*t3.*t12.*1.334512780631337e149+t2.*t4.*t12.*9.186210921246462e143-t3.*t4.*t12.*1.203380298251844e147+t4.*t7.*t8.*9.525104135032322e144-t4.*t7.*t9.*8.818585319188816e144+t4.*t7.*t11.*1.120090641148718e142-t2.*t3.*u_d.*1.469155936798232e149-t4.*t7.*u_d.*2.668960652872475e143-t2.*t3.*u_w.*9.715079228286328e145-t4.*t7.*u_w.*5.290466196739964e147+dalpha_d.*dphi.*t2.*t3.*9.861749103797278e145-dalpha_d.*dphi.*t4.*t7.*1.809020315603064e144+dphi.*dtheta.*t2.*t3.*1.631760804185649e147+dphi.*dtheta.*t4.*t7.*5.300636193719425e148-dphi.*dtheta.*t2.*t10.*7.712138709328024e146+dphi.*dtheta.*t4.*t10.*2.182109992837606e148+dalpha_d.*t2.*t3.*t4.*2.135491664465506e147-t2.*t3.*t4.*t8.*1.68230929579121e145+t2.*t3.*t4.*t9.*8.940574289691811e145-t2.*t3.*t4.*t11.*1.93559876146543e142-t2.*t3.*t4.*u_d.*8.143792311599028e147+dalpha_d.*dtheta.*t2.*t3.*t4.*1.293139560710605e147))./5.0331648e7;(t17.*(dalpha_d.*4.522184942318321e140+dalpha_w.*8.805560806480576e127+t8.*6.154668832694734e142+t9.*2.869935565132996e139-t12.*1.248370016166362e142-u_d.*1.724555313312285e141-u_w.*8.222202527834451e127+dalpha_d.*dphi.*5.878051265226671e138+dphi.*dtheta.*9.751005483471465e139+dalpha_d.*t2.*4.522184939662445e140-dalpha_d.*t3.*2.133085717458765e140-dalpha_d.*t4.*4.657748003804959e135-dalpha_w.*t2.*3.741465503727623e138-dalpha_w.*t3.*6.215771087011924e138+dalpha_w.*t4.*2.032774030125455e140+dalpha_w.*t6.*1.688543971634853e140+t2.*t8.*1.087028177642454e143+t2.*t9.*9.915690262309799e139-t3.*t8.*2.903117928648753e142-t3.*t9.*2.869935565132996e139-t2.*t11.*5.340123394515024e139-t4.*t9.*3.579419271212179e138+t3.*t12.*7.954311270127565e141-t4.*t11.*1.860526847583816e138-t6.*t9.*2.641160491951841e137-t2.*u_d.*1.724555312299455e141+t3.*u_d.*8.134617125827224e140+t4.*u_d.*1.776252888921789e136+t2.*u_w.*3.493597716106183e138+t3.*u_w.*5.803983399496439e138-t4.*u_w.*1.898105087947712e140-t6.*u_w.*1.576679875030551e140-dalpha_d.*dphi.*t3.*5.878053381009897e138-dalpha_d.*dphi.*t6.*5.403738551596732e136+dalpha_d.*dtheta.*t2.*3.195587199125158e135+dalpha_d.*dtheta.*t4.*8.632794085632756e139-dphi.*dtheta.*t3.*2.410149866216753e140+dphi.*dtheta.*t4.*8.429855969237392e140+dphi.*dtheta.*t6.*2.67567640353434e141+dphi.*dtheta.*t10.*9.198974347646347e139-dalpha_d.*t2.*t3.*2.133085717458765e140-dalpha_d.*t4.*t7.*4.171511703990788e135-t2.*t3.*t8.*5.127442397381567e142-t2.*t3.*t9.*9.912462320535689e139+t2.*t3.*t11.*1.337056189493095e129+t2.*t4.*t12.*7.197162340568767e139+t4.*t7.*t9.*5.333190942401371e138-t4.*t7.*t11.*6.836983001915266e133+t2.*t3.*u_d.*8.134617125827224e140+t4.*t7.*u_d.*1.590824516339585e136-dalpha_d.*dtheta.*t2.*t3.*1.507338045722084e135+dalpha_d.*dtheta.*t4.*t7.*7.707712419063812e139+dphi.*dtheta.*t4.*t7.*7.526520950284869e140-dphi.*dtheta.*t2.*t3.*t4.*2.524198896603436e141))./3.0;t17.*(dalpha_d.*-1.953520800495833e134+dalpha_w.*1.729599983981297e141+t9.*1.492208877839942e139-t11.*6.692983518381563e136+u_d.*7.44983833950419e134-u_w.*1.615015973765943e141+dalpha_d.*dtheta.*5.878051265226671e138+dphi.*dtheta.*5.739871130265992e139-dalpha_d.*t2.*2.991839496296188e137-dalpha_d.*t3.*2.987233050784576e137+dalpha_d.*t4.*1.62673261775228e139+dalpha_d.*t6.*8.133672976670994e138+dalpha_w.*t2.*6.082332038541877e141+dalpha_w.*t3.*1.054765219058206e142+dalpha_w.*t4.*3.257921397330419e140+dalpha_w.*t6.*9.805219940868945e139-t2.*t8.*4.074977231722947e139+t2.*t9.*5.537148120454597e139-t3.*t8.*7.197162340568767e139+t3.*t9.*1.108512289555581e140+t4.*t8.*2.213974145089955e141+t4.*t9.*2.546478498260574e141-t2.*t12.*3.766629362213417e141+t3.*t11.*4.613023353263087e139+t4.*t11.*7.526520950284869e140+t6.*t9.*6.493473116806737e140-t4.*t12.*3.357935700107647e142+t6.*t11.*2.010218218769819e140-t9.*t10.*1.857016974366627e140-t10.*t11.*4.599637907627746e139+t2.*u_d.*1.140951280349461e138+t3.*u_d.*1.1391945918938e138-t4.*u_d.*6.203617090115956e139-t6.*u_d.*3.10181231585622e139-t2.*u_w.*5.67938453455705e141-t3.*u_w.*9.848882360825443e141-t4.*u_w.*3.042087850770577e140-t6.*u_w.*9.155635393994624e139+dalpha_d.*dphi.*t2.*5.671611868266142e138+dalpha_d.*dphi.*t3.*1.814627090224519e135+dalpha_d.*dphi.*t4.*2.00859831646309e140+dalpha_d.*dphi.*t6.*4.639511289119264e139-dalpha_d.*dtheta.*t3.*5.878053381009897e138-dalpha_d.*dtheta.*t6.*5.403738551596732e136+dphi.*dtheta.*t2.*1.924357468365363e140-dphi.*dtheta.*t3.*5.739871130426955e139-dphi.*dtheta.*t4.*7.050941940413137e138-dphi.*dtheta.*t6.*5.282732523126947e137+dalpha_w.*t2.*t3.*8.66647608401046e141-dalpha_w.*t4.*t7.*3.257921397330388e140-t2.*t3.*t9.*8.305631449327385e139+t2.*t4.*t8.*3.910287207493707e141+t2.*t3.*t12.*3.772305778530216e141-t2.*t4.*t12.*6.172733385361553e142-t3.*t4.*t12.*1.003480250873935e143-t4.*t7.*t9.*2.302288025836296e141-t4.*t7.*t11.*7.526520950284869e140-t2.*t3.*u_w.*8.092332008306006e141+t4.*t7.*u_w.*3.042087850770547e140-dalpha_d.*dphi.*t2.*t3.*5.667683809167948e138-dalpha_d.*dphi.*t4.*t7.*1.508460218302231e140-dphi.*dtheta.*t2.*t3.*1.923711858887551e140+dphi.*dtheta.*t4.*t7.*1.055855587149976e139+t2.*t3.*t4.*t9.*2.542065703706819e141+t2.*t3.*t4.*t11.*1.223560886221138e141).*(-1.0./3.0);dphi;dtheta];
