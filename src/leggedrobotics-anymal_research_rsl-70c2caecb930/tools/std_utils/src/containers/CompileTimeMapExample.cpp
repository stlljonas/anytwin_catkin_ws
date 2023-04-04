#include <std_utils/std_utils.hpp>

#include <iostream>
#include <memory>

// An enum defining some things
enum class Things : unsigned int {
  CAT = 0,
  GREEN,
  USA,
  DOG,
  ELEPHANT,
  SWITZERLAND,
  RED,
  COMPUTER,
  GERMANY,
  SHEEP,
  BELGIUM,
  IPHONE,
  LAPTOP,
  ENGLAND,
  SWEDEN,
  TABLET,
  DUCK,
  COW,
  BLUE,
  YELLOW,
  SIZE
};

// An enum defining some types
enum class Types : unsigned int { COLOR = 0, COUNTRY, ANIMAL, DEVICE, SIZE };

// Typedef for simple map filling
template <Things key, Types value>
using kv = std_utils::KeyValuePair<Things, Types, key, value>;

// Define the map
using ctm =
    std_utils::CompileTimeMap<Things, Types, kv<Things::CAT, Types::ANIMAL>, kv<Things::GREEN, Types::COLOR>,
                              kv<Things::USA, Types::COUNTRY>, kv<Things::DOG, Types::ANIMAL>, kv<Things::ELEPHANT, Types::ANIMAL>,
                              kv<Things::SWITZERLAND, Types::COUNTRY>, kv<Things::RED, Types::COLOR>, kv<Things::COMPUTER, Types::DEVICE>,
                              kv<Things::GERMANY, Types::COUNTRY>, kv<Things::SHEEP, Types::ANIMAL>, kv<Things::BELGIUM, Types::COUNTRY>,
                              kv<Things::IPHONE, Types::DEVICE>, kv<Things::LAPTOP, Types::DEVICE>, kv<Things::ENGLAND, Types::COUNTRY>,
                              kv<Things::SWEDEN, Types::COUNTRY>, kv<Things::TABLET, Types::DEVICE>, kv<Things::DUCK, Types::ANIMAL>,
                              kv<Things::COW, Types::ANIMAL>, kv<Things::BLUE, Types::COLOR>, kv<Things::YELLOW, Types::COLOR>>;

//! Map with wrong key type int
using ctm_wrongKey = std_utils::CompileTimeMap<Things, Types, std_utils::KeyValuePair<Things, int, Things::CAT, 2>>;

//! Wrong type int within map
using ctm_wrongType = std_utils::CompileTimeMap<Things, Types, kv<Things::CAT, Types::ANIMAL>, int>;

//! Same key multiple times
using ctm_keyNotUnique = std_utils::CompileTimeMap<Things, Types, kv<Things::CAT, Types::ANIMAL>, kv<Things::CAT, Types::ANIMAL>>;

//! No key-value pairs
using ctm_nokeyvalues = std_utils::CompileTimeMap<Things, Types>;

//! **** Prepare Test forEach ****

//! Define pet function
struct Thing {
  virtual ~Thing() = default;
  virtual void pet() {}
};
struct Cat : public Thing {
  void pet() override { std::cout << "Meow!" << std::endl; }
};
struct Dog : public Thing {
  void pet() override { std::cout << "Wuff!" << std::endl; }
};
struct Duck : public Thing {
  void pet() override { std::cout << "Quak!" << std::endl; }
};

//! Map enum to type
template <Things>
struct MapThingToType {
  using type = Thing;
};
template <>
struct MapThingToType<Things::CAT> {
  using type = Cat;
};
template <>
struct MapThingToType<Things::DOG> {
  using type = Dog;
};
template <>
struct MapThingToType<Things::DUCK> {
  using type = Duck;
};

//! Farm of animals
struct AnimalFarm {
  std::array<std::unique_ptr<Thing>, ctm::size()> animals_;

  template <Things thing, Types type>
  struct Pet {
    bool operator()(std::array<std::unique_ptr<Thing>, ctm::size()>& animals) {
      animals[static_cast<int>(thing)] = std::unique_ptr<Thing>(new typename MapThingToType<thing>::type());
      animals[static_cast<int>(thing)]->pet();
      return true;
    }
  };
};

template <Things thing>
struct GetId {
  bool operator()() {
    std::cout << static_cast<int>(thing) << " ";
    return true;
  }
};

using cts = std_utils::CompileTimeSet<Types, Types::COLOR, Types::COUNTRY, Types::ANIMAL, Types::DEVICE>;

int main() {
  /* This won´t compile -> value is of wrong type*/
  //    int a = static_cast<int>(ctm_wrongKey::at<Things::CAT>());

  /* This won´t compile -> wrong type int within map*/
  //    int b = static_cast<int>(ctm_wrongType::at<Things::CAT>());

  /* This won´t compile -> key is not unique*/
  //    int c = static_cast<int>(ctm_keyNotUnique::at<Things::CAT>());

  /* This won´t compile -> no key value pairs*/
  //    int d = static_cast<int>(ctm_nokeyvalues::at<Things::CAT>());

  /* These will all compile */
  int e = static_cast<int>(ctm::at(Things::CAT)) + static_cast<int>(ctm::at(Things::GREEN)) + static_cast<int>(ctm::at(Things::USA)) +
          static_cast<int>(ctm::at(Things::DOG)) + static_cast<int>(ctm::at(Things::ELEPHANT)) + static_cast<int>(ctm::at(Things::RED)) +
          static_cast<int>(ctm::at(Things::SWITZERLAND));
  std::cout << "Count pet value: " << e << std::endl;

  // Test forEach
  AnimalFarm farm;
  std::cout << "Pet all things: " << std::endl;
  ctm::forEach<AnimalFarm::Pet>(farm.animals_);
  std::cout << "Pet all things from green to red: " << std::endl;
  ctm::forEachRange<AnimalFarm::Pet>(Things::GREEN, Things::RED, farm.animals_);

  // Check if all values are smaller than 10
  std::cout << "All Things smaller 10: " << std::boolalpha
            << ctm::forEach([](const Things& thing, const Types& /*type*/) { return static_cast<int>(thing) < 10; }) << std::endl;

  std::cout << "All Things smaller 10 from green to red: " << std::boolalpha
            << ctm::forEachRange(Things::GREEN, Things::RED,
                                 [](const Things& thing, const Types& /*type*/) { return static_cast<int>(thing) < 10; })
            << std::endl;

  std::cout << "All Animal Ids:" << std::endl;
  using ctsAnimals = ctm::findAll<Types::ANIMAL>;
  ctsAnimals::forEach<GetId>();
  std::cout << std::endl;

  std::cout << "All Thing Ids:" << std::endl;
  using ctsThings = std_utils::cts_from_enum_t<Things, Things::SIZE>;
  ctsThings::forEach<GetId>();
  std::cout << std::endl;

  std::cout << "All Thing Ids In Range (2,8):" << std::endl;
  using ctsSeq = std_utils::cts_from_sequence_t<Things, std_utils::make_index_sequence_range_t<2, 8>>;
  ctsSeq::forEach<GetId>();
  std::cout << std::endl;

  std::cout << "Build map from subenum:" << std::endl;
  using ctmSubenum = std_utils::ctm_from_subenum_t<Things, Types, 4>;
  ctmSubenum::forEach([](const Types& type, const Things& thing) {
    std::cout << "Type: " << static_cast<int>(type) << " maps to thing: " << static_cast<int>(thing) << "." << std::endl;
    return true;
  });

  std::cout << "Transform map from subenum:" << std::endl;
  CONSECUTIVE_ENUM(ThingSub, SubGreen, SubCat);
  using ctm_thingsub = std_utils::CompileTimeMap<ThingSub, Things, std_utils::KeyValuePair<ThingSub, Things, ThingSub::SubCat, Things::CAT>,
                                                 std_utils::KeyValuePair<ThingSub, Things, ThingSub::SubGreen, Things::GREEN>>;
  CONSECUTIVE_ENUM(TypeSub, SubAnimal, SubColor);
  using ctm_typesub = std_utils::CompileTimeMap<TypeSub, Types, std_utils::KeyValuePair<TypeSub, Types, TypeSub::SubAnimal, Types::ANIMAL>,
                                                std_utils::KeyValuePair<TypeSub, Types, TypeSub::SubColor, Types::COLOR>>;
  using ctm_thingtypesub =
      std_utils::CompileTimeMap<ThingSub, TypeSub, std_utils::KeyValuePair<ThingSub, TypeSub, ThingSub::SubGreen, TypeSub::SubColor>,
                                std_utils::KeyValuePair<ThingSub, TypeSub, ThingSub::SubCat, TypeSub::SubAnimal>>;
  using ctm_thingtype_fromsub = std_utils::ctm_transform_t<Things, Types, ctm_thingtypesub, ctm_thingsub, ctm_typesub>;
  std::cout << static_cast<int>(ctm_thingtypesub::at(ThingSub::SubCat)) << std::endl;
  std::cout << static_cast<int>(ctm_thingtype_fromsub::at(Things::CAT)) << std::endl;

  std::cout << "Pet Map1:" << std::endl;
  using ctm1 = std_utils::CompileTimeMap<Things, Types, kv<Things::GREEN, Types::COLOR>>;
  ctm1::forEach<AnimalFarm::Pet>(farm.animals_);
  std::cout << "Pet Map2:" << std::endl;
  using ctm2 = std_utils::CompileTimeMap<Things, Types, kv<Things::CAT, Types::ANIMAL>>;
  ctm2::forEach<AnimalFarm::Pet>(farm.animals_);
  std::cout << "Pet Concatenated Map12:" << std::endl;
  using ctm12 = std_utils::ctm_concatenate_t<Things, Types, ctm1, ctm2>;
  //  using ctm12fail = std_utils::concatenate_ctm_t<int, double>;
  ctm12::forEach<AnimalFarm::Pet>(farm.animals_);
  std::cout << "Pet Map123 with added KV:" << std::endl;
  using ctm123 = std_utils::ctm_insert_back_t<Things, Types, ctm12, kv<Things::DUCK, Types::ANIMAL>>;
  //  using ctm123fail = std_utils::ctm_addKV_t<ctm12, double>;
  ctm123::forEach<AnimalFarm::Pet>(farm.animals_);

  // Key set, Value set
  std::cout << "Pet Map from sets:" << std::endl;
  using cts_keys = std_utils::CompileTimeSet<Things, Things::GREEN, Things::USA, Things::CAT, Things::LAPTOP>;
  using cts_values = std_utils::CompileTimeSet<Types, Types::COLOR, Types::COUNTRY, Types::ANIMAL, Types::DEVICE>;
  using ctm_sets = std_utils::ctm_from_sets_t<Things, Types, cts_keys, cts_values>;
  ctm_sets::forEach([](const Things& thing, const Types& /*type*/) {
    std::cout << static_cast<int>(thing) << std::endl;
    return true;
  });

  // Add to ctm values
  using ctm_add_prev = std_utils::CompileTimeMap<TypeSub, int, std_utils::KeyValuePair<TypeSub, int, TypeSub::SubAnimal, 2>,
                                                 std_utils::KeyValuePair<TypeSub, int, TypeSub::SubColor, 3>>;
  using ctm_add_after = std_utils::ctm_add_to_values_t<TypeSub, int, ctm_add_prev, 2>;
  std::cout << "Add 2 to values:" << std::endl;
  std::cout << ctm_add_prev::at(TypeSub::SubAnimal) << " " << ctm_add_prev::at(TypeSub::SubColor) << std::endl;
  std::cout << ctm_add_after::at(TypeSub::SubAnimal) << " " << ctm_add_after::at(TypeSub::SubColor) << std::endl;

  // Compare to unordered map
  std_utils::EnumMap<Things, Types> unorderedMap;
  unorderedMap.insert(std::make_pair(Things::CAT, Types::ANIMAL));
  unorderedMap.insert(std::make_pair(Things::GREEN, Types::COLOR));
  unorderedMap.insert(std::make_pair(Things::USA, Types::COUNTRY));
  unorderedMap.insert(std::make_pair(Things::DOG, Types::ANIMAL));
  unorderedMap.insert(std::make_pair(Things::ELEPHANT, Types::ANIMAL));
  unorderedMap.insert(std::make_pair(Things::SWITZERLAND, Types::COUNTRY));
  unorderedMap.insert(std::make_pair(Things::RED, Types::COLOR));
  unorderedMap.insert(std::make_pair(Things::COMPUTER, Types::DEVICE));
  unorderedMap.insert(std::make_pair(Things::GERMANY, Types::COUNTRY));
  unorderedMap.insert(std::make_pair(Things::SHEEP, Types::ANIMAL));
  unorderedMap.insert(std::make_pair(Things::BELGIUM, Types::COUNTRY));
  unorderedMap.insert(std::make_pair(Things::IPHONE, Types::DEVICE));
  unorderedMap.insert(std::make_pair(Things::LAPTOP, Types::DEVICE));
  unorderedMap.insert(std::make_pair(Things::ENGLAND, Types::COUNTRY));
  unorderedMap.insert(std::make_pair(Things::SWEDEN, Types::COUNTRY));
  unorderedMap.insert(std::make_pair(Things::TABLET, Types::DEVICE));
  unorderedMap.insert(std::make_pair(Things::DUCK, Types::ANIMAL));
  unorderedMap.insert(std::make_pair(Things::COW, Types::ANIMAL));
  unorderedMap.insert(std::make_pair(Things::BLUE, Types::COLOR));
  unorderedMap.insert(std::make_pair(Things::YELLOW, Types::COLOR));

  std_utils::HighResolutionClockTimer timer;
  unsigned int c = 0;
  timer.pinTime();
  for (unsigned int i = 0; i < 100000; ++i) {
    c += i * static_cast<unsigned int>(unorderedMap.at(Things::CAT));
  }
  auto um = timer.getElapsedTimeNanoSec();
  c = 0;
  timer.pinTime();
  for (unsigned int i = 0; i < 100000; ++i) {
    c += i * static_cast<unsigned int>(ctm::at(Things::CAT));
  }
  auto ctc = timer.getElapsedTimeNanoSec();
  std::cout << "Known element CAT // Unordered Map took: " << um << " ns. Constexpr CTM took: " << ctc << " ns." << std::endl;

  c = 0;
  timer.pinTime();
  for (unsigned int i = 0; i < 100000; ++i) {
    c += i * static_cast<unsigned int>(unorderedMap.at(static_cast<Things>(rand() % 20)));
  }
  um = timer.getElapsedTimeNanoSec();
  c = 0;
  timer.pinTime();
  for (unsigned int i = 0; i < 100000; ++i) {
    c += i * static_cast<unsigned int>(ctm::at(static_cast<Things>(rand() % 20)));
  }
  ctc = timer.getElapsedTimeNanoSec();
  std::cout << "Known element CAT // Unordered Map took: " << um << " ns. Constexpr CTM took: " << ctc << " ns." << std::endl;

  // auto oor = ctm::at<Things::SIZE>(); Would not compile key out of range
  try {
    auto oor2 = ctm::at(Things::SIZE);  // Throw std::SIZE exception
    std::cout << "oor 2: " << static_cast<int>(oor2);
  } catch (std::out_of_range& e) {
  }

  // auto oor3 = ctm::find<Types::SIZE>(); // Would not compile key out of range
  try {
    auto oor4 = ctm::find(Types::SIZE);  // Throw std::SIZE exception
    std::cout << "oor 4: " << static_cast<int>(oor4);
  } catch (std::out_of_range& e) {
  }

  return 0;
}
