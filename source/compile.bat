@echo off
cls
echo ********************************************************************************************************************
printf "Check main.c\n\n"
cpplint.py --extensions=hpp,cpp,h,c --filter=-legal/copyright main.c
echo ********************************************************************************************************************

printf "Check attitudedetermination/ekf_sun_vector.c\n\n"
cpplint.py --extensions=hpp,cpp,h,c --filter=-legal/copyright attitudedetermination/ekf_sun_vector.c
echo ********************************************************************************************************************

printf "Check attitudedetermination/propagate_state.c\n\n"
cpplint.py --extensions=hpp,cpp,h,c --filter=-legal/copyright attitudedetermination/propagate_state.c
echo ********************************************************************************************************************

printf "Check attitudedetermination/wahba_triad_weight.c\n\n"
cpplint.py --extensions=hpp,cpp,h,c --filter=-legal/copyright attitudedetermination/wahba_triad_weight.c
echo ********************************************************************************************************************

printf "Check control/detumbling.c\n\n"
cpplint.py --extensions=hpp,cpp,h,c --filter=-legal/copyright control/detumbling.c
echo ********************************************************************************************************************

printf "Check control/sun_pointing_kf.c\n\n"
cpplint.py --extensions=hpp,cpp,h,c --filter=-legal/copyright control/sun_pointing_kf.c
echo ********************************************************************************************************************

printf "Check control/sun_pointing_raw.c\n\n"
cpplint.py --extensions=hpp,cpp,h,c --filter=-legal/copyright control/sun_pointing_raw.c
echo ********************************************************************************************************************

printf "Check data/igrf_data.c\n\n"
cpplint.py --extensions=hpp,cpp,h,c --filter=-legal/copyright data/igrf_data.c
echo ********************************************************************************************************************

printf "Check geomagneticfield/ass_legendre.c\n\n"
cpplint.py --extensions=hpp,cpp,h,c --filter=-legal/copyright geomagneticfield/ass_legendre.c
echo ********************************************************************************************************************

printf "Check geomagneticfield/deriv_legendre.c\n\n"
cpplint.py --extensions=hpp,cpp,h,c --filter=-legal/copyright geomagneticfield/deriv_legendre.c
echo ********************************************************************************************************************

printf "Check geomagneticfield/igrf.c\n\n"
cpplint.py --extensions=hpp,cpp,h,c --filter=-legal/copyright geomagneticfield/igrf.c
echo ********************************************************************************************************************

printf "Check geomagneticfield/legendrepoly.c\n\n"
cpplint.py --extensions=hpp,cpp,h,c --filter=-legal/copyright geomagneticfield/legendrepoly.c
echo ********************************************************************************************************************

printf "Check geomagneticfield/sch_legendre.c\n\n"
cpplint.py --extensions=hpp,cpp,h,c --filter=-legal/copyright geomagneticfield/sch_legendre.c
echo ********************************************************************************************************************

printf "Check header/attitude_determination.h\n\n"
cpplint.py --extensions=hpp,cpp,h,c --filter=-legal/copyright header/attitude_determination.h
echo ********************************************************************************************************************

printf "Check header/debug_pack.h\n\n"
cpplint.py --extensions=hpp,cpp,h,c --filter=-legal/copyright header/debug_pack.h
echo ********************************************************************************************************************

printf "Check header/control.h\n\n"
cpplint.py --extensions=hpp,cpp,h,c --filter=-legal/copyright header/control.h
echo ********************************************************************************************************************

printf "Check header/geomagnetic_field.h\n\n"
cpplint.py --extensions=hpp,cpp,h,c --filter=-legal/copyright header/geomagnetic_field.h
echo ********************************************************************************************************************

printf "Check header/main.h\n\n"
cpplint.py --extensions=hpp,cpp,h,c --filter=-legal/copyright header/main.h
echo ********************************************************************************************************************

printf "Check header/math_pack.h\n\n"
cpplint.py --extensions=hpp,cpp,h,c --filter=-legal/copyright header/math_pack.h
echo ********************************************************************************************************************

printf "Check header/structurs.h\n\n"
cpplint.py --extensions=hpp,cpp,h,c --filter=-legal/copyright header/structurs.h
echo ********************************************************************************************************************

printf "Check lib/math_pack.c\n\n"
cpplint.py --extensions=hpp,cpp,h,c --filter=-legal/copyright lib/math_pack.c
echo ********************************************************************************************************************

printf "Check lib/debug_pack.c\n\n"
cpplint.py --extensions=hpp,cpp,h,c --filter=-legal/copyright lib/debug_pack.c
echo ********************************************************************************************************************
echo ********************************************************************************************************************
printf "START COMPILE main.c\n\n"
gcc main.c -o build/pw-sat2-adcs
printf "END COMPILE main.c\n"
echo ********************************************************************************************************************
echo ********************************************************************************************************************