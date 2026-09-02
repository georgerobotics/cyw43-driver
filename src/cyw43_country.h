/*
 * This file is part of the cyw43-driver
 *
 * Copyright (C) 2019-2026 George Robotics Pty Ltd
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef CYW43_INCLUDED_CYW43_COUNTRY_H
#define CYW43_INCLUDED_CYW43_COUNTRY_H

/** \addtogroup cyw43_driver
 */
//!\{

/**
 *  \file cyw43_country.h
 *  \brief CYW43 country codes
 */

/*!
 * \brief create a country code from the two character country and revision number
 */
#define CYW43_COUNTRY(A, B, REV) ((unsigned char)(A) | ((unsigned char)(B) << 8) | ((REV) << 16))


/*!
 * \name Country codes
 * \anchor CYW43_COUNTRY_
 */
//!\{

// Worldwide Locale (passive Ch12-14)
#define CYW43_COUNTRY_WORLDWIDE         CYW43_COUNTRY('X', 'X', 0)

#define CYW43_COUNTRY_AUSTRALIA         CYW43_COUNTRY('A', 'U', 0)
#define CYW43_COUNTRY_AUSTRIA           CYW43_COUNTRY('A', 'T', 0)
#define CYW43_COUNTRY_BELGIUM           CYW43_COUNTRY('B', 'E', 0)
#define CYW43_COUNTRY_BRAZIL            CYW43_COUNTRY('B', 'R', 0)
#define CYW43_COUNTRY_CANADA            CYW43_COUNTRY('C', 'A', 0)
#define CYW43_COUNTRY_CHILE             CYW43_COUNTRY('C', 'L', 0)
#define CYW43_COUNTRY_CHINA             CYW43_COUNTRY('C', 'N', 0)
#define CYW43_COUNTRY_COLOMBIA          CYW43_COUNTRY('C', 'O', 0)
#define CYW43_COUNTRY_CZECH_REPUBLIC    CYW43_COUNTRY('C', 'Z', 0)
#define CYW43_COUNTRY_DENMARK           CYW43_COUNTRY('D', 'K', 0)
#define CYW43_COUNTRY_ESTONIA           CYW43_COUNTRY('E', 'E', 0)
#define CYW43_COUNTRY_FINLAND           CYW43_COUNTRY('F', 'I', 0)
#define CYW43_COUNTRY_FRANCE            CYW43_COUNTRY('F', 'R', 0)
#define CYW43_COUNTRY_GERMANY           CYW43_COUNTRY('D', 'E', 0)
#define CYW43_COUNTRY_GREECE            CYW43_COUNTRY('G', 'R', 0)
#define CYW43_COUNTRY_HONG_KONG         CYW43_COUNTRY('H', 'K', 0)
#define CYW43_COUNTRY_HUNGARY           CYW43_COUNTRY('H', 'U', 0)
#define CYW43_COUNTRY_ICELAND           CYW43_COUNTRY('I', 'S', 0)
#define CYW43_COUNTRY_INDIA             CYW43_COUNTRY('I', 'N', 0)
#define CYW43_COUNTRY_ISRAEL            CYW43_COUNTRY('I', 'L', 0)
#define CYW43_COUNTRY_ITALY             CYW43_COUNTRY('I', 'T', 0)
#define CYW43_COUNTRY_JAPAN             CYW43_COUNTRY('J', 'P', 0)
#define CYW43_COUNTRY_KENYA             CYW43_COUNTRY('K', 'E', 0)
#define CYW43_COUNTRY_LATVIA            CYW43_COUNTRY('L', 'V', 0)
#define CYW43_COUNTRY_LIECHTENSTEIN     CYW43_COUNTRY('L', 'I', 0)
#define CYW43_COUNTRY_LITHUANIA         CYW43_COUNTRY('L', 'T', 0)
#define CYW43_COUNTRY_LUXEMBOURG        CYW43_COUNTRY('L', 'U', 0)
#define CYW43_COUNTRY_MALAYSIA          CYW43_COUNTRY('M', 'Y', 0)
#define CYW43_COUNTRY_MALTA             CYW43_COUNTRY('M', 'T', 0)
#define CYW43_COUNTRY_MEXICO            CYW43_COUNTRY('M', 'X', 0)
#define CYW43_COUNTRY_NETHERLANDS       CYW43_COUNTRY('N', 'L', 0)
#define CYW43_COUNTRY_NEW_ZEALAND       CYW43_COUNTRY('N', 'Z', 0)
#define CYW43_COUNTRY_NIGERIA           CYW43_COUNTRY('N', 'G', 0)
#define CYW43_COUNTRY_NORWAY            CYW43_COUNTRY('N', 'O', 0)
#define CYW43_COUNTRY_PERU              CYW43_COUNTRY('P', 'E', 0)
#define CYW43_COUNTRY_PHILIPPINES       CYW43_COUNTRY('P', 'H', 0)
#define CYW43_COUNTRY_POLAND            CYW43_COUNTRY('P', 'L', 0)
#define CYW43_COUNTRY_PORTUGAL          CYW43_COUNTRY('P', 'T', 0)
#define CYW43_COUNTRY_SINGAPORE         CYW43_COUNTRY('S', 'G', 0)
#define CYW43_COUNTRY_SLOVAKIA          CYW43_COUNTRY('S', 'K', 0)
#define CYW43_COUNTRY_SLOVENIA          CYW43_COUNTRY('S', 'I', 0)
#define CYW43_COUNTRY_SOUTH_AFRICA      CYW43_COUNTRY('Z', 'A', 0)
#define CYW43_COUNTRY_SOUTH_KOREA       CYW43_COUNTRY('K', 'R', 0)
#define CYW43_COUNTRY_SPAIN             CYW43_COUNTRY('E', 'S', 0)
#define CYW43_COUNTRY_SWEDEN            CYW43_COUNTRY('S', 'E', 0)
#define CYW43_COUNTRY_SWITZERLAND       CYW43_COUNTRY('C', 'H', 0)
#define CYW43_COUNTRY_TAIWAN            CYW43_COUNTRY('T', 'W', 0)
#define CYW43_COUNTRY_THAILAND          CYW43_COUNTRY('T', 'H', 0)
#define CYW43_COUNTRY_TURKEY            CYW43_COUNTRY('T', 'R', 0)
#define CYW43_COUNTRY_UK                CYW43_COUNTRY('G', 'B', 0)
#define CYW43_COUNTRY_USA               CYW43_COUNTRY('U', 'S', 0)

//!\}

//!\}

#endif
